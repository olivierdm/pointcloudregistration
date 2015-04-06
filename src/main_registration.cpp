/**
@file main_registration.cpp
*/
#include "ros/ros.h"
#include <image_transport/image_transport.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <image_transport/subscriber_filter.h>
#include "boost/thread.hpp"
#include <boost/lexical_cast.hpp>
#include "pointcloudregistration/pcl_registration.h"
#include "pointcloudregistration/pcl_analyser.h"
#include "pointcloudregistration/vision.h"
#include "pointcloudregistration/settings.h"

#include <Eigen/Geometry>
#include <iostream>

PCL_registration* registrar=0;
PCL_analyser* pcl_analyse=0;
KeyFrameGraph* graph=0;
Vision* visor=0;
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
void callback(const sensor_msgs::ImageConstPtr& imgMsg ,const lsd_slam_viewer::keyframeMsg::ConstPtr &frameMsg, const sensor_msgs::ImuConstPtr &imuMsg)
{
	if(pcl_analyse==0 || visor==0||!firstKF)
		return;
	//check if both are ready
	if(!(visor->ready()) || !(pcl_analyse->ready()))
		return;
	//pass the data and do processing
	ROS_INFO("in callback");
	visor->process(imgMsg);
	pcl_analyse->process(frameMsg);
}
void callback(const sensor_msgs::ImageConstPtr& imgMsg ,const lsd_slam_viewer::keyframeMsg::ConstPtr &frameMsg)
{
	if(pcl_analyse==0 || visor==0||!firstKF)
		return;
	//check if both are ready
	if(!(visor->ready()) || !(pcl_analyse->ready()))
		return;
	//pass the data and do processing
	ROS_INFO("in callback");
	visor->process(imgMsg);
	pcl_analyse->process(frameMsg);
}
void rosThreadLoop()
{
///
/// \brief This void function takes care of handling the different subscribers and is started on startup.
///
/// A set of ROS filters are coupled to obtain time syncronization between the IMU, camera and the pose estimated by lsd-slam.
/// The approximate time sync policy is chosen because there is a difference in frequency between the different topics
///
	printf("Started ROS thread\n");


	ROS_INFO("registrar node starting");

	ros::NodeHandle nh;
	image_transport::ImageTransport it(nh);
	ros::Subscriber keyFrames_sub = nh.subscribe(nh.resolveName("lsd_slam/keyframes"),20, frameCb);
	ros::Subscriber graph_sub  = nh.subscribe(nh.resolveName("lsd_slam/graph"),10, graphCb);
	image_transport::SubscriberFilter image_sub(it, nh.resolveName("image"),1);
	message_filters::Subscriber<lsd_slam_viewer::keyframeMsg> liveFrames_sub(nh,nh.resolveName("lsd_slam/liveframes"), 1);
	message_filters::Subscriber<sensor_msgs::Imu> imu_sub(nh,nh.resolveName("/ardrone/imu"),1);
	typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, lsd_slam_viewer::keyframeMsg, sensor_msgs::Imu> MySyncPolicy;
	typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, lsd_slam_viewer::keyframeMsg> MySyncPolicy1;
	//ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
	//message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), image_sub, liveFrames_sub, imu_sub);
	//sync.registerCallback(boost::bind(&callback, _1, _2,_3));
	message_filters::Synchronizer<MySyncPolicy1> sync1(MySyncPolicy1(10), image_sub, liveFrames_sub);
	sync1.registerCallback(boost::bind(&callback, _1, _2));
	ROS_INFO("subscribers initialized - going for a spin");
	ros::spin();
	ros::shutdown();
	printf("Exiting ROS thread\n");
	//exit(1);
}


int main( int argc, char** argv )
{
	boost::thread rosThread;
	// start ROS thread
	ros::init(argc, argv, "registrar");
	rosThread = boost::thread(rosThreadLoop);

	visor = new Vision();
	graph = new KeyFrameGraph();
	registrar = new PCL_registration(graph);
	pcl_analyse = new PCL_analyser(graph);
	rosThread.join();
	std::cout<<"Shutting down ... " << std::endl;
	ros::shutdown();
	delete registrar;
	//can not be deleted causes lock at shutdown
	//delete visor;
	//delete pcl_analyse;
	//delete graph;
	std::cout << "Done" << std::endl;

}
