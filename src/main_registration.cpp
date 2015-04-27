/**
@file main_registration.cpp
*/
#include "ros/ros.h"
#include <ros/callback_queue.h>
#include <image_transport/image_transport.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <image_transport/subscriber_filter.h>
//#include "boost/thread.hpp"
//#include <boost/lexical_cast.hpp>
#include "pointcloudregistration/pcl_registration.h"
#include "pointcloudregistration/pcl_analyser.h"
#include "pointcloudregistration/vision.h"
#include "pointcloudregistration/linereg.h"
#include "pointcloudregistration/settings.h"
#include <Eigen/Geometry>
#include <iostream>
#include "tbb/task_group.h"

PCL_registration* registrar=0;
PCL_analyser* pcl_analyse=0;
KeyFrameGraph* graph=0;
Vision* visor=0;
LineReg* stairs=0;
tbb::task_group g;
bool firstKF=false;

void frameCb(lsd_slam_viewer::keyframeMsgConstPtr msg)
{
	if(msg->time > lastFrameTime || registrar == 0 ) return;
	firstKF=true;
	registrar->addFrameMsg(msg);
}
void graphCb(lsd_slam_viewer::keyframeGraphMsgConstPtr msg)
{
	if(registrar == 0) return;
	registrar->addGraphMsg(msg);
}
void callback(const sensor_msgs::ImageConstPtr& imgMsg ,const lsd_slam_viewer::keyframeMsgConstPtr &frameMsg, const tum_ardrone::filter_stateConstPtr &poseMsg)
{
	if(pcl_analyse==0 || visor==0||!firstKF)
		return;
	cv::UMat depthImg, H, CI;
	//pass the data and do processing
	ROS_INFO("in callback");
	g.wait();
	g.run([&]{(*pcl_analyse)(frameMsg, depthImg, H, CI);});
	std::vector<cv::Rect> rectangles;
        std::vector<cv::Vec4f> lines;
	(*visor)(imgMsg,poseMsg, rectangles, lines);
	g.wait();
(*stairs)(depthImg, H, CI, rectangles, lines, imgMsg, frameMsg, poseMsg);
	//g.run([&]{(*stairs)(depthImg, H, CI, rectangles, lines, imgMsg, frameMsg, poseMsg);});
	//pcl_analyse->process(frameMsg);
}

int main( int argc, char** argv )
{
	boost::thread rosThread;
	// start ROS thread
	ros::init(argc, argv, "registrar");
	//rosThread = boost::thread(rosThreadLoop);
	graph = new KeyFrameGraph();
	registrar = new PCL_registration(graph);
	stairs = new LineReg(registrar);
	visor = new Vision(stairs);
	pcl_analyse = new PCL_analyser(graph);

	ros::CallbackQueue lsdqueue;
	ros::NodeHandle lsdhandler;
	lsdhandler.setCallbackQueue(&lsdqueue);
	ros::Subscriber keyFrames_sub = lsdhandler.subscribe(lsdhandler.resolveName("lsd_slam/keyframes"),5, frameCb);
	ros::Subscriber graph_sub  = lsdhandler.subscribe(lsdhandler.resolveName("lsd_slam/graph"),1, graphCb);
	ros::AsyncSpinner lsdspinner(1, &lsdqueue);
	lsdspinner.start();

	ros::CallbackQueue processqueue;
	ros::NodeHandle processhandler;
	processhandler.setCallbackQueue(&processqueue);
	image_transport::ImageTransport it(processhandler);
	image_transport::SubscriberFilter image_sub(it, processhandler.resolveName("image"),1);
	message_filters::Subscriber<lsd_slam_viewer::keyframeMsg> liveFrames_sub(processhandler,processhandler.resolveName("lsd_slam/liveframes"), 1);
	message_filters::Subscriber<tum_ardrone::filter_state> pose_sub(processhandler,processhandler.resolveName("/ardrone/predictedPose"),1);
	typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, lsd_slam_viewer::keyframeMsg, tum_ardrone::filter_state> MySyncPolicy;
	//ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
	message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(2), image_sub, liveFrames_sub, pose_sub);
	sync.registerCallback(boost::bind(&callback, _1, _2,_3));
	ros::AsyncSpinner processspinner(1, &processqueue);
	processspinner.start();

	ros::waitForShutdown();
	//rosThread.join();
	std::cout<<"Shutting down ... " << std::endl;
	delete registrar;
	//can not be deleted causes lock at shutdown
	//delete visor;
	//delete pcl_analyse;
	//delete graph;
	std::cout << "Done" << std::endl;

}
