/**
@file main_registration.cpp
*/
#include "ros/ros.h"
#include <ros/callback_queue.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <image_transport/subscriber_filter.h>
#include "pointcloudregistration/pcl_registration.h"
#include "pointcloudregistration/pcl_analyser.h"
#include "pointcloudregistration/KeyFrameGraph.h"
#include "pointcloudregistration/vision.h"
#include "pointcloudregistration/linereg.h"
#include "pointcloudregistration/settings.h"
#include <Eigen/Geometry>
#include <iostream>
#include <memory>
#include "tbb/task_group.h"

std::unique_ptr<PCL_registration> registrar;
std::unique_ptr<PCL_analyser> pcl_analyse;
std::shared_ptr<KeyFrameGraph> graph;
std::unique_ptr<Vision> visor;
std::unique_ptr<LineReg> stairs;
tbb::task_group g;

bool firstKF=false;
void frameCb(lsd_slam_viewer::keyframeMsgConstPtr msg)
{
	if(msg->time > lastFrameTime ) return;
	firstKF=true;
	registrar->addFrameMsg(msg);
}
void graphCb(lsd_slam_viewer::keyframeGraphMsgConstPtr msg)
{
	registrar->addGraphMsg(msg);
}
void callback(const sensor_msgs::ImageConstPtr& imgMsg ,const lsd_slam_viewer::keyframeMsgConstPtr &frameMsg, const tum_ardrone::filter_stateConstPtr &poseMsg)
{
///
/// \brief Process the incoming data and detect stairs
/// @param[in] imgMsg ros image msg from the ardrone front camera
/// @param[in] frameMsg lsdslam liveframe
/// @param[in] poseMsg pose information from TUM_ardrone

	cv::UMat depthImg, H, CI;
	std::vector<cv::Rect> rectangles;
	std::vector<cv::Vec4f> lines;
	cv_bridge::CvImagePtr cv_input_ptr;
	pcl::PointCloud<pcl::PointXYZ>::Ptr canPlane;
	Eigen::Affine3f pose;

	//pass the data and do processing
	ROS_DEBUG("in callback");
	std::vector<std::shared_ptr<KeyFrame>> keyframes = graph->getFrames();
	g.run([&]{(*pcl_analyse)(frameMsg, keyframes, depthImg, H, CI);});
	try{
		cv_input_ptr = cv_bridge::toCvCopy(imgMsg, sensor_msgs::image_encodings::BGR8);
	}
	catch (cv_bridge::Exception& e){
		ROS_ERROR_STREAM("cv_bridge exception: " << e.what());
		return;
	}
	(*visor)(cv_input_ptr, poseMsg, rectangles, lines);
	g.wait();
	if((*stairs)(depthImg, H, CI, rectangles, lines, cv_input_ptr, frameMsg, poseMsg));
		registrar->drawPlane(canPlane, pose);
}

int main( int argc, char** argv )
{
	//init ros
	ros::init(argc, argv, "registrar");
	//init classes for input
	graph = std::make_shared<KeyFrameGraph>();
	registrar.reset(new PCL_registration(graph));
	//init queu and handler for lsdslam messages
	ros::CallbackQueue lsdqueue;
	ros::NodeHandle lsdhandler;
	lsdhandler.setCallbackQueue(&lsdqueue);
	ros::Subscriber keyFrames_sub = lsdhandler.subscribe(lsdhandler.resolveName("lsd_slam/keyframes"), 20, frameCb);
	ros::Subscriber liveFrames_subl = lsdhandler.subscribe(lsdhandler.resolveName("lsd_slam/liveframes"), 1, frameCb);	
	ros::Subscriber graph_sub  = lsdhandler.subscribe(lsdhandler.resolveName("lsd_slam/graph"), 10, graphCb);
	ros::AsyncSpinner lsdspinner(1, &lsdqueue);
	lsdspinner.start();

	//init classes for processing
	stairs.reset(new LineReg());
	visor.reset(new Vision());
	pcl_analyse.reset(new PCL_analyser());
	//init queu and 
	ros::CallbackQueue processqueue;
	ros::NodeHandle processhandler;
	processhandler.setCallbackQueue(&processqueue);
	image_transport::ImageTransport it(processhandler);
	image_transport::SubscriberFilter image_sub(it, processhandler.resolveName("image"), 1);
	message_filters::Subscriber<lsd_slam_viewer::keyframeMsg> liveFrames_sub(processhandler,processhandler.resolveName("lsd_slam/liveframes"), 1);
	message_filters::Subscriber<tum_ardrone::filter_state> pose_sub(processhandler,processhandler.resolveName("/ardrone/predictedPose"),1);
	typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, lsd_slam_viewer::keyframeMsg, tum_ardrone::filter_state> MySyncPolicy;
	//ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(1), only realtime
	message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(1), image_sub, liveFrames_sub, pose_sub);
	sync.registerCallback(boost::bind(&callback, _1, _2,_3));
	ros::AsyncSpinner processspinner(1, &processqueue);
	processspinner.start();

	ros::waitForShutdown();
	std::cout<<"Shutting down ... " << std::endl;
}
