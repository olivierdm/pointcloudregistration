#include "pointcloudregistration/pcl_analyser.h"
#include "pointcloudregistration/KeyFrame.h"
#include "pointcloudregistration/KeyFrameGraph.h"
#include "pointcloudregistration/settings.h"
#include "pcl/point_types.h"
#include <pcl/common/transforms.h>
#include <sophus/se3.hpp>
#include <algorithm>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include "opencv2/core/utility.hpp"
#include <opencv2/imgproc/imgproc.hpp>



PCL_analyser::PCL_analyser(KeyFrameGraph* keyGraph): cloud (new PointCloud), depth(new PointCloud),nh("~"),it(nh),it2(nh),graph(keyGraph)
{
	pub = it.advertise("depth",1);
	pub2 = it2.advertise("curvature",1);
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
/*
PointCloud::Ptr PCL_analyser::getCloud()
{
	cloudMtx.lock();
	return cloud;
}*/
/*void PCL_analyser::release()
{
	cloudMtx.unlock();
}*/
void PCL_analyser::getDepthImage()
{
///
/// \brief Generates a depth image from the received keyframes
///
/// The function gets the currently received keyframes from the keyFrameGraph instance and transforms them to
/// coordinates in the liveframe axes. This set of accumulated points is then projected using the camera parameters
/// to get a depth image that corresponds with the image captured by camera.
///
	std::vector<KeyFrame*> keyframes = graph->getFrames();
	int x,y,num(0);
	float pixel;

	boost::mutex::scoped_lock lock(cloudMtx);
	cloud->clear();
	depthImg.setTo(maxZ);
	for(std::size_t i=0;i<keyframes.size();i++)
	{
		//get keyframe in analysed keyframe
		soph = (camToWorld.inverse()*keyframes[i]->camToWorld).matrix();
		pcl::transformPointCloud(*keyframes[i]->getPCL(),*depth,soph);
		keyframes[i]->release();
		*cloud+=*depth;
		ROS_INFO_STREAM("cloud: "<< keyframes[i]->id);
	}
 	assert(width == depthImg.cols && height == depthImg.rows);
	for(PointCloud::iterator it = cloud->begin(); it != cloud->end(); it++){ 
		if(it->z>maxZ|| it->z<minZ)
			continue;
		x=static_cast<int> ((fx*(it->x)/(it->z)+cx)+0.5f);
		y=static_cast<int> ((fy*(it->y)/(it->z)+cy)+0.5f);

		if(x<0 || x >= width||y<0||y >= height)
			continue;
		num++;
		depthImg.at<float>(y,x)=std::min(depthImg.at<float>(y,x),it->z);
    	}

	double minVal,maxVal;
	//cv::medianBlur(depthImg.getUMat(cv::ACCESS_READ,cv::USAGE_ALLOCATE_DEVICE_MEMORY), filt, 2);
	//cv::GaussianBlur(depthImg.getUMat(cv::ACCESS_READ,cv::USAGE_ALLOCATE_DEVICE_MEMORY),filt,cv::Size(5,5),0);
	//cv::resize(filt,filt,cv::Size(0,0),1.0f/my_scaleDepthImage,1.0f/my_scaleDepthImage,cv::INTER_AREA);
	cv::minMaxLoc(depthImg, &minVal, &maxVal);
	cv_bridge::CvImagePtr cv_ptr(new cv_bridge::CvImage);
	cv::Mat colored;
	cv::Mat structElm = cv::getStructuringElement(cv::MORPH_RECT,cv::Size(7,7),cv::Point(-1,-1));
	cv::erode(depthImg.getUMat(cv::ACCESS_READ), filt,structElm,cv::Point(-1,-1),1);
	cv::resize(filt,filt,cv::Size(0,0),1.0f/my_scaleDepthImage,1.0f/my_scaleDepthImage,cv::INTER_AREA);
	filt.convertTo(colored,CV_8UC1,256.0/maxZ);
	applyColorMap(colored,cv_ptr->image, cv::COLORMAP_AUTUMN);
	//cv::imwrite("/home/rosuser/Downloads/depth.png",cv_ptr->image);
	//cv::normalize(depthImg,cv_ptr->image,0,256,cv::NORM_MINMAX,CV_8UC1);
	//cv::normalize(filt.getMat(cv::ACCESS_READ),cv_ptr->image,0,256,cv::NORM_MINMAX,CV_8UC1);
	cv_ptr->header.stamp=header.stamp;
	cv_ptr->encoding="bgr8";
	msg = cv_ptr->toImageMsg();
	ROS_INFO_STREAM("sending message, "<< num << " points: , total pixs: "<< cloud->width << " min: "<< minVal << " max: "<< maxVal);
	pub.publish(msg);
}
void PCL_analyser::process(lsd_slam_viewer::keyframeMsgConstPtr msg)
{
///
/// \brief  Accepts the data of a liveframe message and copies the neccesary data to the adequate private variables.
/// @param[in] msg a liveframe message
/// 
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
///
/// \brief This method is started in a different thread and waits for data. Upon reception of new data the methods
/// PCL_analyser::getDepthImage() and PCL_analyser::calcCurvature() are called.
///
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
///
/// \brief calc the curvature using the constructed depth image.
///
/// The sobel operator calculates the different derivatives applying gaussian smoothing.
	cv::UMat hx, hy, hxx, hyy, hxy,hx2,hy2,Kden,Knom,K,H,CI,CInom,CIden,Hden,mask32,mask8,Hnom1,Hnom2,Hnom3;
	cv::Sobel(filt,hx,CV_64F,1,0,5);
	cv::Sobel(filt,hy,CV_64F,0,1,5);
	cv::Sobel(filt,hxx,CV_64F,2,0,5);
	cv::Sobel(filt,hyy,CV_64F,0,2,5);
	cv::Sobel(filt,hxy,CV_64F,1,1,5);
	cv::pow(hx,2,hx2);
	cv::pow(hy,2,hy2);
//////// calculate K
	//1+hx^2 + hy^2
	cv::addWeighted(hx2,1.0,hy2,1.0,1.0,Kden);
	//power with non int values need treatment for negative values because it takes the power of the absolute value
	cv::compare(Kden,0.0,mask32,cv::CMP_LT);
	mask32.convertTo(mask8,CV_8U);
	cv::pow(Kden,1.5,Hden);
	cv::subtract(0.0, Hden, Hden, mask8);
	cv::pow(Kden,2,Kden);
	//hxx*hyy-hxy^2
	cv::subtract(hxx.mul(hyy),hxy.mul(hxy),Knom);
	cv::divide(Knom,Kden,K);
//////// calculate H
	//1+hx^2
	cv::add(1.0,hx2,Hnom1);
	//2*hx*hy*hxy
	cv::multiply(2.0,hx.mul(hy).mul(hxy),Hnom2);
	//1+hy^2
	cv::add(1.0,hy2,Hnom3);
	//(1+hx^2)*hyy+2*hx*hy*hxy
	cv::add(Hnom1.mul(hyy),Hnom2,Hnom1);
	//(1+hx^2)*hyy+2*hx*hy*hxy + (1+hy^2)*hxx
	cv::add(Hnom1,Hnom3.mul(hxx),Hnom1);
	cv::divide(Hnom1,Hden,H);
	cv::multiply(0.5,H,H);
//////// calculate CI
	cv::compare(K,0.0,mask32,cv::CMP_GE);
	mask32.convertTo(mask8,CV_8U);
	cv::pow(H,2,H);
	CInom=H.clone();
	//H^2 - K
	cv::subtract(CInom,K,CInom,mask8);
	double eps=1.0;
	//H-eps/2
	cv::subtract(H,eps/2.0,CIden);
	cv::bitwise_not(mask8,mask8);
	//H-eps/2-(K+eps/2)
	cv::add(K,eps/2.0,K);
	cv::subtract(CIden,K,CIden,mask8);
	cv::divide(CInom,CIden,CI);
	ROS_INFO("Sending curvature");
	cv_bridge::CvImagePtr cv_ptr(new cv_bridge::CvImage);
	cv::normalize(hx.getMat(cv::ACCESS_READ),cv_ptr->image,0,255,cv::NORM_MINMAX,CV_8UC1);
	applyColorMap(cv_ptr->image,cv_ptr->image, cv::COLORMAP_AUTUMN);
	//cv::imwrite("/home/rosuser/Downloads/curvature.png",cv_ptr->image);
	cv_ptr->header.stamp=header.stamp;
	cv_ptr->encoding="bgr8";
	msg = cv_ptr->toImageMsg();
	pub2.publish(msg);
}
bool PCL_analyser::ready()
{
///
/// \brief Checks if the frameMutex is still locked and thus the worker thread still occupied with the previous task
///
	boost::mutex::scoped_lock lock(frameMutex, boost::try_to_lock);
	if(lock)
	{
		return true;
	}else{
		return false;
	}
}
