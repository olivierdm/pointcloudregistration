#include "pointcloudregistration/pcl_analyser.h"
#include "pointcloudregistration/settings.h"
#include "pcl/point_types.h"
#include <pcl/common/transforms.h>
#include <sophus/se3.hpp>
#include <algorithm>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>

PCL_analyser::PCL_analyser(KeyFrameGraph* keyGraph): cloud (new PointCloud), depth(new PointCloud), boundingbox(new PointCloud),nh("~"),it(nh),graph(keyGraph)
{
	pub = it.advertise("depth",1);
	wantExit=false;
	data_ready=false;
	//create thread
	worker = boost::thread(boost::bind(&PCL_analyser::threadLoop,this));
	//ctor
}
PCL_analyser::~PCL_analyser()
{
	//destroy thread
	{
		boost::mutex::scoped_lock lock(frameMutex);
		wantExit=true;
		newData.notify_one();
	}
	worker.join();
	//dtor
}
PointCloud::Ptr PCL_analyser::getCloud()
{
	cloudMtx.lock();
	return cloud;
}
void PCL_analyser::release()
{
	cloudMtx.unlock();
}
void PCL_analyser::getDepthImage()
{
	std::vector<KeyFrame*> keyframes = graph->getFrames();
	int x,y,num(0);
	float pixel;

	boost::mutex::scoped_lock lock(cloudMtx);
	cloud->clear();
	depthImg.setTo(maxZ*1.2f);
	for(std::size_t i=0;i<keyframes.size();i++)
	{
		//get keyframe in analysed keyframe
		soph = (camToWorld.inverse()*keyframes[i]->camToWorld).matrix();
		pcl::transformPointCloud(*keyframes[i]->getPCL(),*depth,soph);
		keyframes[i]->release();
		*cloud+=*depth;
	}

	assert(width == depthImg.cols && height == depthImg.rows);
	for(PointCloud::iterator it = cloud->begin(); it != cloud->end(); it++){ 
		if(it->z>maxZ|| it->z<minZ)
			continue;
		x=static_cast<int> ((fx*(it->x)/(it->z)+cx)+0.5f);
		y=static_cast<int> ((fy*(it->y)/(it->z)+cy)+0.5f);

		if(x<0 || x >= width||y<0||y >= height)
			continue;
		pixel=depthImg.at<float>(y,x);
		if(pixel==0.0f){
		depthImg.at<float>(y,x)= (it->z);
		num++;
		}else{
		depthImg.at<float>(y,x)=std::min(pixel,it->z);
		}

    	}

	double minVal,maxVal;
	cv::Mat filt;
	cv::medianBlur(depthImg, filt, my_scaleDepthImage);
	cv::minMaxLoc(depthImg, &minVal, &maxVal);
	
	cv_bridge::CvImagePtr cv_ptr(new cv_bridge::CvImage);
	cv::normalize(filt,cv_ptr->image,0,256,cv::NORM_MINMAX,CV_8UC1);
	cv_ptr->header.stamp=header.stamp;
	cv_ptr->encoding="mono8";
	msg = cv_ptr->toImageMsg();
	ROS_DEBUG_STREAM("sending message, "<< num << " points, min: "<< minVal << " max: "<< maxVal);
	pub.publish(msg);
}
void PCL_analyser::process(lsd_slam_viewer::keyframeMsgConstPtr msg)
{
	//accept the new data
	{
		boost::mutex::scoped_lock lock(frameMutex);
		memcpy(camToWorld.data(), msg->camToWorld.data(), 7*sizeof(float));
		my_scaleDepthImage= static_cast<int> (scaleDepthImage +0.5f);
		width=my_scaleDepthImage*msg->width;
		height=my_scaleDepthImage*msg->height;
		fx=my_scaleDepthImage*msg->fx;
		fy=my_scaleDepthImage*msg->fy;
		cx=my_scaleDepthImage*msg->cx;
		cy=my_scaleDepthImage*msg->cy;
		depthImg.create(height,width,CV_32F);//reinitializes if needed
		//copy header
		header=msg->header;
		data_ready=true;
		ROS_INFO_STREAM("fx: "<< fx << ", fy: "<< fy << ", cx: "<< cx << ", cy: " << cy);
	}
	//notify thread
	newData.notify_one();
}
void PCL_analyser::threadLoop()
{
	ROS_INFO("pcl analyser thread started");
	while(true)
	{
		boost::mutex::scoped_lock lock(frameMutex);
		while(!data_ready)//check if new message passed
		{
			newData.wait(lock);
		}
		data_ready=false;
		if(wantExit)
			return;
		ROS_INFO("starting detection loop");
		getDepthImage();
		calcCurvature();
	}
}
void PCL_analyser::calcCurvature()
{
	ROS_INFO("dummy calc curvature");
}
bool PCL_analyser::ready()
{
	boost::mutex::scoped_lock lock(frameMutex, boost::try_to_lock);
	if(lock)
	{
		return true;
	}else{
		return false;
	}
}
