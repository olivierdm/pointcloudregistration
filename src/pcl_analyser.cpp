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
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <fstream>
#include <string>


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
/*
	cv::Mat output;
	depthImg.convertTo(output,CV_16UC1,65535/maxZ);
	std::vector<int> params;
	params.push_back(CV_IMWRITE_PNG_COMPRESSION);
	params.push_back(0);
	cv::imwrite("/home/rosuser/Downloads/depth/"+ boost::to_string(header.seq) +".png",output,params);*/
	cv::minMaxLoc(depthImg, &minVal, &maxVal);
	cv_bridge::CvImagePtr cv_ptr(new cv_bridge::CvImage);
	cv::Mat colored;
	depthImg.convertTo(colored,CV_8UC1,255.0/maxZ);
	applyColorMap(colored,cv_ptr->image, cv::COLORMAP_AUTUMN);
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
		filterDepth();
		calcCurvature();
	}
}
void PCL_analyser::filterDepth()
{
	cv::UMat filt_f=depthImg.getUMat(cv::ACCESS_READ);
	filt_f.convertTo(filt,CV_64F);
	cv::Mat structElm = cv::getStructuringElement(cv::MORPH_RECT,cv::Size(7,7),cv::Point(-1,-1));
	cv::erode(depthImg.getUMat(cv::ACCESS_READ), filt,structElm,cv::Point(-1,-1),1);
	cv::resize(filt,filt,cv::Size(0,0),1.0f/my_scaleDepthImage,1.0f/my_scaleDepthImage,cv::INTER_AREA);
	cv::GaussianBlur(filt,filt,cv::Size(5,5),2.0);
}
void PCL_analyser::calcCurvature()
{
///
/// \brief calc the curvature using the constructed depth image.
///
/// Calculates filter kernels and applies them.
//create kernels
	int ksize=5;
	cv::UMat kx10, ky10, kx01,ky01,kx11,ky11,kx20,ky20,kx02,ky02;
	cv::getDerivKernels(kx10, ky10, 1, 0, ksize, true, CV_64F );
	cv::getDerivKernels(kx01, ky01, 0, 1, ksize, true, CV_64F );
	cv::getDerivKernels(kx11, ky11, 1, 1, ksize, true, CV_64F );
	cv::getDerivKernels(kx20, ky20, 2, 0, ksize, true, CV_64F );
	cv::getDerivKernels(kx02, ky02, 0, 2, ksize, true, CV_64F );
//filter image
	cv::UMat hx, hy, hxx, hyy, hxy,hx2,hy2,normx,normy;
	cv::sepFilter2D(filt, hx, CV_64F, kx10, ky10);
	cv::sepFilter2D(filt, hy, CV_64F, kx01, ky01);
	cv::sepFilter2D(filt, hxy, CV_64F, kx11, ky11);
	cv::sepFilter2D(filt, hxx, CV_64F, kx20, ky20);
	cv::sepFilter2D(filt, hyy, CV_64F, kx02, ky02);
	cv::divide(static_cast<double>(fx),filt,normx,1.0,CV_64F);
	cv::divide(static_cast<double>(fy),filt,normy,1.0,CV_64F);
//normalize derivatives
	cv::multiply(normx,hx,hx);
	cv::multiply(normx,hy,hy);
	cv::multiply(normx.mul(normx),hxx,hxx);
	cv::multiply(normy.mul(normy),hyy,hyy);
	cv::multiply(normy.mul(normx),hxy,hxy);
	cv::UMat Kden,Knom,K,H,CI,CInom,CIden,Hden,mask32,mask8,Hnom1,Hnom2,Hnom3;

	cv::pow(hx,2,hx2);
	cv::pow(hy,2,hy2);
//////// calculate K
	//1+hx^2 + hy^2
	cv::addWeighted(hx2,1.0,hy2,1.0,1.0,Kden);
	//power with non int values need treatment for negative values because it takes the power of the absolute value
	cv::pow(Kden,1.5,Hden);
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
	cv::subtract(Hnom1.mul(hyy),Hnom2,Hnom1);
	//(1+hx^2)*hyy+2*hx*hy*hxy + (1+hy^2)*hxx
	cv::add(Hnom1,Hnom3.mul(hxx),Hnom1);
	cv::divide(Hnom1,Hden,H);
	cv::multiply(0.5,H,H);
//////// calculate CI
	cv::Mat structElm = cv::getStructuringElement(cv::MORPH_RECT,cv::Size(3,3),cv::Point(-1,-1));
	//cv::dilate(H, H,structElm,cv::Point(-1,-1),1);
	//cv::dilate(K, K,structElm,cv::Point(-1,-1),1);
	cv::compare(K,0.0,mask32,cv::CMP_GE);
	mask32.convertTo(mask8,CV_8UC1);
	cv::pow(H,2,H);
	CInom=H.clone();
	//H^2 - K
	cv::subtract(CInom,K,CInom,mask8);
	double eps=1.0;
	//H-eps
	cv::subtract(H,eps,CIden);
	//cv::imwrite("/home/rosuser/Downloads/mask1.png",mask8);
	cv::bitwise_not(mask8,mask8);
	//cv::imwrite("/home/rosuser/Downloads/mask2.png",mask8);
	//H-eps-K
	cv::subtract(CIden,K,CIden,mask8);
	cv::divide(CInom,CIden,CI);
	ROS_INFO("Sending curvature");
	cv_bridge::CvImagePtr cv_ptr(new cv_bridge::CvImage);
	double minVal,maxVal;
	cv::minMaxLoc(CI, &minVal, &maxVal);
	writeHist(-0.1f,1.1f,48,CI);


	cv::compare(CI,0.0,mask32,cv::CMP_LT);
	int bad = cv::countNonZero(mask32);
	CI.setTo(0.0,mask32);
	cv::compare(CI,1.0,mask32,cv::CMP_GT);
	bad+= cv::countNonZero(mask32);
	CI.setTo(1.0,mask32);
	ROS_INFO_STREAM("min: "<< minVal << "max: "<< maxVal << " deleted: " << bad << " size " << cv::countNonZero(CI));
	CI.convertTo(cv_ptr->image,CV_8UC1,255.0);
	//cv::normalize(CIv.getMat(cv::ACCESS_READ),cv_ptr->image,0,255,cv::NORM_MINMAX,CV_8UC1);
	applyColorMap(cv_ptr->image,cv_ptr->image, cv::COLORMAP_AUTUMN);
	//cv::imwrite("/home/rosuser/Downloads/K.png",K);
	//cv::imwrite("/home/rosuser/Downloads/H.png",H);
	cv_ptr->header.stamp=header.stamp;
	cv_ptr->encoding="bgr8";
	msg = cv_ptr->toImageMsg();
	pub2.publish(msg);
}
void PCL_analyser::writeHist(float min, float max, int bins,cv::UMat CI)
{
	float ciranges[] = { min, max};
	const float* ranges[] ={ciranges};
	cv::MatND hist;
    // we compute the histogram from the 0-th channel
	cv::Mat CIhist;
	CI.convertTo(CIhist,CV_32FC1);
    	cv::calcHist( &CIhist, 1, 0, cv::Mat(), // do not use mask
             hist, 1,&bins, ranges,
             true, // the histogram is uniform
             false );
	std::ofstream myfile;
	myfile.open ("histogram.csv",std::ofstream::out | std::ofstream::app);
	myfile << "\"histogram- min\"," << min << ",\"max:\", " << max << std::endl;
	float step = (max-min)/bins;
	myfile<< min;
	for( int h = 1; h < bins; h++ )
         {
            float binVal = min + h*step;
           myfile<<", "<<binVal;
         }
	myfile<< std::endl<<hist.at<float>(0);
	for( int h = 1; h < bins; h++ )
         {
            float binVal = hist.at<float>(h);
           myfile<<", "<<binVal;
         }
	myfile<< std::endl;
	myfile.close();
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
