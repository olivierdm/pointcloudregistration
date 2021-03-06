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
#include "tbb/parallel_invoke.h"


std::unique_ptr<PCL_registration> registrar;
std::unique_ptr<PCL_analyser> pcl_analyse;
std::shared_ptr<KeyFrameGraph> graph;
std::unique_ptr<Vision> visor;
std::unique_ptr<LineReg> stairs;
tbb::task_group g;

bool firstKF=false;
void frameCb(lsd_slam_viewer::keyframeMsgConstPtr msg)
{
/// \brief Handles the liveframes and the keyframes to construct the keyframe graph.
/// @param[in] msg liveframe message or keyframe message
	if(msg->time > lastFrameTime ) return;
	firstKF=true;
	registrar->addFrameMsg(msg);
}
void graphCb(lsd_slam_viewer::keyframeGraphMsgConstPtr msg)
{
/// \brief Handles graph messages to update the constraints and pose of the keyframes already in the graph.
/// @param[in] msg keyframe graph message
	registrar->addGraphMsg(msg);
}
void callback(const sensor_msgs::ImageConstPtr& imgMsg ,const lsd_slam_viewer::keyframeMsgConstPtr &frameMsg)
{
///
/// \brief Process the incoming data and detect stairs
/// @param[in] imgMsg ros image msg from the ardrone front camera
/// @param[in] frameMsg lsdslam liveframe


/// Initialize variables that will be passed by reference.
	cv::UMat depthImg, H, CI;
	std::vector<cv::Rect> rectangles;
	std::vector<cv::Vec4f> lines;
	cv_bridge::CvImagePtr cv_input_ptr;
	if(registrar->addFrameMsg(frameMsg))
		return;

/// Retrieve the image from the image message.
	try{
		cv_input_ptr = cv_bridge::toCvCopy(imgMsg, sensor_msgs::image_encodings::BGR8);
	}
	catch (cv_bridge::Exception& e){
		ROS_ERROR_STREAM("cv_bridge exception: " << e.what());
		return;
	}
/// Get the keyframes from the graph and pass the data to the PCL_analyser. Image processing is done in parallel
	std::vector<std::shared_ptr<KeyFrame>> keyframes = graph->getFrames();
	tbb::parallel_invoke([&]{(*pcl_analyse)(frameMsg, keyframes, depthImg, H, CI);},
			[&]{(*visor)(cv_input_ptr, rectangles, lines);}
	);

/// Combine the data and attempt detection
	g.run([=]{pcl::PointCloud<pcl::PointXYZ>::Ptr canPlane(new pcl::PointCloud<pcl::PointXYZ>);
		Eigen::Affine3f pose;
		if((*stairs)(depthImg, H, CI, rectangles, lines, cv_input_ptr, frameMsg, canPlane, pose))
		registrar->drawPlane(canPlane, pose);});
}

int main( int argc, char** argv )
{
///
/// \brief Initialize the main program.
/// @param[in] argc number of arguments passed
/// @param[in] argv parameters passed to the main function, can contain parameters for ros initialisation

/// Initialise ros, this needs to be done before adding subscribers.
	ros::init(argc, argv, "registrar");
/// Initialise classes for handling the input.
	graph = std::make_shared<KeyFrameGraph>();
	registrar.reset(new PCL_registration(graph));
/// Initialize queu and handler for lsdslam messages.
	ros::CallbackQueue lsdqueue;
	ros::NodeHandle lsdhandler;
	lsdhandler.setCallbackQueue(&lsdqueue);
	ros::Subscriber keyFrames_sub = lsdhandler.subscribe(lsdhandler.resolveName("lsd_slam/keyframes"), 20, frameCb);
	//ros::Subscriber liveFrames_subl = lsdhandler.subscribe(lsdhandler.resolveName("lsd_slam/liveframes"), 1, frameCb);	
	ros::Subscriber graph_sub  = lsdhandler.subscribe(lsdhandler.resolveName("lsd_slam/graph"), 10, graphCb);
	ros::AsyncSpinner lsdspinner(1, &lsdqueue);
	lsdspinner.start();
/// Initialize classes for processing.
	stairs.reset(new LineReg());
	visor.reset(new Vision());
	pcl_analyse.reset(new PCL_analyser());
/// Initialize queu and handler for processing
	ros::CallbackQueue processqueue;
	ros::NodeHandle processhandler;
	processhandler.setCallbackQueue(&processqueue);
	image_transport::ImageTransport it(processhandler);
	image_transport::SubscriberFilter image_sub(it, processhandler.resolveName("image"), 1);
	message_filters::Subscriber<lsd_slam_viewer::keyframeMsg> liveFrames_sub(processhandler,processhandler.resolveName("lsd_slam/liveframes"), 1);
	typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, lsd_slam_viewer::keyframeMsg> MySyncPolicy;
	//ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(1), only realtime
	message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(1), image_sub, liveFrames_sub/*, pose_sub*/);
	sync.registerCallback(boost::bind(&callback, _1, _2/*,_3*/));
	ros::AsyncSpinner processspinner(1, &processqueue);
	processspinner.start();

	ros::waitForShutdown();
	std::cout<<"Shutting down ... " << std::endl;
}
