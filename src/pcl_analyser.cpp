#include "pointcloudregistration/pcl_analyser.h"
#include "pointcloudregistration/KeyFrame.h"
#include "pointcloudregistration/KeyFrameGraph.h"
#include "pointcloudregistration/settings.h"
#include "pointcloudregistration/linereg.h"
#include <Eigen/Geometry>
#include "pcl/point_types.h"
#include <pcl/common/transforms.h>
#include <sophus/se3.hpp>
#include <algorithm>
#include <opencv2/opencv.hpp>
#include "opencv2/core/utility.hpp"
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <fstream>
#include <string>
#include <chrono>


struct framedist{
	framedist(KeyFrame* key,Sophus::Sim3f camToWorld):frame(key),camcenter(0.0f,0.0f,1.0f,1.0f)
	{
		soph = (camToWorld.inverse()*frame->camToWorld).matrix();
		camcenterlive = soph*camcenter;
		dist=(camcenterlive-camcenter).squaredNorm();
		
	}
	KeyFrame* frame;
	Eigen::Vector4f camcenter,camcenterlive;
	float dist;
	Eigen::Matrix4f soph;
	bool operator<(const framedist& rhs) const
		{return dist < rhs.dist;}
};
PCL_analyser::PCL_analyser(KeyFrameGraph* keyGraph): cloud (new PointCloud), depth(new PointCloud), nh("~"),graph(keyGraph),it(nh),cv_depth_ptr(new cv_bridge::CvImage),cv_depthf_ptr(new cv_bridge::CvImage),cv_H_ptr(new cv_bridge::CvImage), cv_K_ptr(new cv_bridge::CvImage), cv_CI_ptr(new cv_bridge::CvImage)
{
	pub_depth = it.advertise("depth",1);
	pub_depthf = it.advertise("depth_filtered",1);
	pub_curv = it.advertise("curvature",1);
	pub_K = it.advertise("K",1);
	pub_H = it.advertise("H",1);
	wantExit=false;
	data_ready=false;
	//initiate depth image
	cv_depth_ptr->encoding="mono16";
	cv_depth_ptr->header.frame_id="ardrone_base_frontcam";
	//initiate filtered depth image
	cv_depthf_ptr->encoding="mono16";
	cv_depthf_ptr->header.frame_id="ardrone_base_frontcam";
	//initiate H image
	cv_H_ptr->encoding="mono16";
	cv_H_ptr->header.frame_id="mean_curvature";
	//initiate K image
	cv_K_ptr->encoding="mono16";
	cv_K_ptr->header.frame_id="gaussian_curvature";
	//initiate curvature image
	cv_CI_ptr->encoding="mono16";
	cv_CI_ptr->header.frame_id="curvature";
	//create thread
	//worker = boost::thread(boost::bind(&PCL_analyser::threadLoop,this));

	//ctor
}
PCL_analyser::~PCL_analyser()
{
	//destroy thread
	{
		boost::mutex::scoped_lock lock(frameMutex);
		wantExit=true;
	}
	//worker.join();
	//dtor
}


void PCL_analyser::operator ()(lsd_slam_viewer::keyframeMsgConstPtr msg, cv::UMat & filt, cv::UMat & H, cv::UMat & CI)
{
		memcpy(camToWorld.data(), msg->camToWorld.data(), 7*sizeof(float));
		my_scaleDepthImage= static_cast<int> (scaleDepthImage +0.5f);
		width=my_scaleDepthImage*msg->width;
		height=my_scaleDepthImage*msg->height;
		fx=my_scaleDepthImage*msg->fx;
		fy=my_scaleDepthImage*msg->fy;
		cx=my_scaleDepthImage*msg->cx;
		cy=my_scaleDepthImage*msg->cy;
		//depthImg.create(height,width,CV_32F);//reinitializes if needed
		//copy header
		header=msg->header;
		data_ready=true;
		ROS_INFO_STREAM("fx: "<< fx << ", fy: "<< fy << ", cx: "<< cx << ", cy: " << cy);
		ROS_INFO("starting detection loop");
		cv::Mat depthImg(height,width,CV_32F);
		getDepthImage(depthImg);
		filterDepth(depthImg,filt);
		calcCurvature(filt,H,CI);
		/*cv::Vec4f camera;
		camera[0]=fx/my_scaleDepthImage;
		camera[1]=fy/my_scaleDepthImage;
		camera[2]=cx/my_scaleDepthImage;
		camera[3]=cy/my_scaleDepthImage;
		Eigen::Affine3f trans;
		trans=camToWorld.matrix();
		stairs->process(CI.getMat(cv::ACCESS_READ),filt.getMat(cv::ACCESS_READ).clone(),H.getMat(cv::ACCESS_READ).clone(),camera,trans);*/

}
void PCL_analyser::getDepthImage(cv::Mat & depthImg)
{
///
/// \brief Generates a depth image from the received keyframes
///
/// The function gets the currently received keyframes from the keyFrameGraph instance and transforms them to
/// coordinates in the liveframe axes. This set of accumulated points is then projected using the camera parameters
/// to get a depth image that corresponds with the image captured by camera.
///
	std::vector<KeyFrame*> keyframes = graph->getFrames();
	int x,y;
	boost::mutex::scoped_lock lock(cloudMtx);
	cloud->clear();
	depthImg.setTo(maxZ);
	//need to insert function that calculates clouds that are in region of interest
	//select randomly
	std::random_shuffle(keyframes.begin(),keyframes.end());
	//int k=0;
	std::vector<framedist> mykeyframes;

	for(std::size_t i=0;i<keyframes.size();i++)
	{
		mykeyframes.push_back(framedist(keyframes[i],camToWorld));
	}
	std::sort(mykeyframes.begin(),mykeyframes.end());
	int k = std::min(10,static_cast<int>(mykeyframes.size()));
	mykeyframes.erase(mykeyframes.begin()+k,mykeyframes.end());

	/*std::random_shuffle(mykeyframes.begin(),mykeyframes.end());
	k = std::min(5,static_cast<int>(mykeyframes.size()));
	mykeyframes.erase(mykeyframes.begin()+k,mykeyframes.end());*/
	auto t1 = std::chrono::high_resolution_clock::now();
	for(std::size_t i=0;i<mykeyframes.size();i++)
	{
		//get keyframe in analysed keyframe
		pcl::transformPointCloud(*mykeyframes[i].frame->getPCL(),*depth,mykeyframes[i].soph);
		mykeyframes[i].frame->release();
		*cloud+=*depth;
		ROS_INFO_STREAM("cloud: "<< mykeyframes[i].frame->id);
	}
  auto t2 = std::chrono::high_resolution_clock::now();

ROS_INFO_STREAM("acumulating "<< k << "clouds took "<< std::chrono::duration_cast<std::chrono::milliseconds>(t2-t1).count() <<" milisecond(s).");
 	assert(width == depthImg.cols && height == depthImg.rows);
t1 = std::chrono::high_resolution_clock::now();
	for(PointCloud::iterator it = cloud->begin(); it != cloud->end(); it++){ 
		if(it->z>maxZ|| it->z<minZ)
			continue;
		x=static_cast<int> ((fx*(it->x)/(it->z)+cx)+0.5f);
		y=static_cast<int> ((fy*(it->y)/(it->z)+cy)+0.5f);

		if(x<0 || x >= width||y<0||y >= height)
			continue;
		depthImg.at<float>(y,x)=std::min(depthImg.at<float>(y,x),it->z);
    	}

t2 = std::chrono::high_resolution_clock::now();
ROS_INFO_STREAM("projection took "<<  std::chrono::duration_cast<std::chrono::milliseconds>(t2-t1).count() <<" milisecond(s).");
	if (!pub_depth.getNumSubscribers())
		return;
	depthImg.convertTo(cv_depth_ptr->image,CV_16UC1,1000);
	cv_depth_ptr->header.stamp=header.stamp;
	pub_depth.publish(cv_depth_ptr->toImageMsg());
}
/*void PCL_analyser::process(lsd_slam_viewer::keyframeMsgConstPtr msg)
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
}*/
/*void PCL_analyser::threadLoop()
{
///
/// \brief This method is started in a different thread and waits for data. Upon reception of new data the methods
/// PCL_analyser::getDepthImage(), PCL_analyser::filterDepth() and PCL_analyser::calcCurvature() are called.
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
		cv::Vec4f camera;
		camera[0]=fx/my_scaleDepthImage;
		camera[1]=fy/my_scaleDepthImage;
		camera[2]=cx/my_scaleDepthImage;
		camera[3]=cy/my_scaleDepthImage;
		Eigen::Affine3f trans;
		trans=camToWorld.matrix();
		stairs->process(CI.getMat(cv::ACCESS_READ),filt.getMat(cv::ACCESS_READ).clone(),H.getMat(cv::ACCESS_READ).clone(),camera,trans);
	}
}*/
void PCL_analyser::filterDepth(cv::Mat & depthImg, cv::UMat & filt)
{
///
/// \brief applies required filtering and resizes the image back to the original size.
/// Applies first median filter to remove outliers and afterwards region growing to file the holes. Gaussian smoothing is applied to make the derivatis stable.
///
	cv::UMat filt_t, mask;
	cv::compare(depthImg,maxZ,mask,cv::CMP_EQ);
	cv::Mat structElm = cv::getStructuringElement(cv::MORPH_ELLIPSE,cv::Size(struct_x*my_scaleDepthImage,struct_y*my_scaleDepthImage),cv::Point(-1,-1));
	cv::erode(depthImg.getUMat(cv::ACCESS_READ), filt_t,structElm,cv::Point(-1,-1),1);
	filt_t.copyTo(filt,mask);
	cv::resize(filt.clone(),filt,cv::Size(0,0),1.0f/my_scaleDepthImage,1.0f/my_scaleDepthImage,cv::INTER_AREA);
	cv::GaussianBlur(filt.clone(),filt,cv::Size(gauss_size,gauss_size),gauss_sigma);

	if (!pub_depthf.getNumSubscribers())
		return;
	filt.convertTo(cv_depthf_ptr->image,CV_16UC1,1000);
	cv_depthf_ptr->header.stamp=header.stamp;
	pub_depthf.publish(cv_depthf_ptr->toImageMsg());
}
void PCL_analyser::calcCurvature(const cv::UMat & filt, cv::UMat & H, cv::UMat & CI)
{
///
/// \brief calc the curvature using the constructed depth image.
///
/// Calculates filter kernels and applies them. Secondly calculates Gaussian and Mean curvature. After that these values are used to calculate the curvature index. The different cases (K \geq 0, K < 0) are handled using masks.
///

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
//normalize derivatives
	cv::divide(static_cast<double>(fx/my_scaleDepthImage),filt,normx,1.0,CV_64F);
	cv::divide(static_cast<double>(fy/my_scaleDepthImage),filt,normy,1.0,CV_64F);
	cv::multiply(normx,hx,hx);
	cv::multiply(normx,hy,hy);
	cv::multiply(normx.mul(normx),hxx,hxx);
	cv::multiply(normy.mul(normy),hyy,hyy);
	cv::multiply(normy.mul(normx),hxy,hxy);
	cv::UMat Kden,Knom,K,CInom,CIden,Hden,mask32,Hnom1,Hnom2,Hnom3;

	cv::pow(hx,2,hx2);
	cv::pow(hy,2,hy2);
//////// calculate K
	//1+hx^2 + hy^2
	cv::addWeighted(hx2,1.0,hy2,1.0,1.0,Kden);
	//power with non int values need treatment for negative values because it takes the power of the absolute value, this is not needed here as argument is always positive.
	cv::pow(Kden,1.5,Hden);
	cv::pow(Kden.clone(),2,Kden);
	//hxx*hyy-hxy^2
	cv::subtract(hxx.mul(hyy),hxy.mul(hxy),Knom);
	cv::divide(Knom,Kden,K);

	//publish K
	if (pub_K.getNumSubscribers() != 0)
	{
	K.convertTo(cv_K_ptr->image,CV_16UC1,1000.0);
	cv_K_ptr->header.stamp=header.stamp;
	pub_K.publish(cv_K_ptr->toImageMsg());
	}
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
	cv::add(Hnom1.clone(),Hnom3.mul(hxx),Hnom1);
	cv::divide(Hnom1,Hden,H);
	cv::multiply(0.5,H.clone(),H);

	//publish H
	if (pub_H.getNumSubscribers() != 0)
	{
	H.convertTo(cv_H_ptr->image,CV_16UC1,1000.0);
	cv_H_ptr->header.stamp=header.stamp;
	pub_H.publish(cv_H_ptr->toImageMsg());
	}
//////// calculate CI
	cv::compare(K,0.0,mask32,cv::CMP_GE);
	cv::pow(H.clone(),2,H);
	CInom=H.clone();
	//H^2 - K
	cv::subtract(CInom.clone(),K,CInom,mask32);
	double eps=1.0;
	//H-eps
	cv::subtract(H,eps,CIden);
	cv::bitwise_not(mask32,mask32);
	//H-eps-K
	cv::subtract(CIden.clone(),K,CIden,mask32);
	cv::divide(CInom,CIden,CI);
	//writeHist(-20.0f,10.0f,48,CI);
	cv::compare(CI,0.0,mask32,cv::CMP_LT);
	CI.setTo(0.0,mask32);
	cv::compare(CI,1.0,mask32,cv::CMP_GT);
	CI.setTo(1.0,mask32);
	//publish curvature
	if (pub_curv.getNumSubscribers() != 0)
	{
	CI.convertTo(cv_CI_ptr->image,CV_16UC1,100.0);
	cv_CI_ptr->header.stamp=header.stamp;
	pub_curv.publish(cv_CI_ptr->toImageMsg());
	}
}
void PCL_analyser::writeHist(float min, float max, int bins,cv::UMat CI)
{
///
/// \brief Writes histogram to csv file for debug purposes.
///
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
/// \brief Checks if the frameMutex is still locked and thus the worker thread still occupied with the previous task.
///
	boost::mutex::scoped_lock lock(frameMutex, boost::try_to_lock);
	if(lock)
	{
		return true;
	}else{
		return false;
	}
}
