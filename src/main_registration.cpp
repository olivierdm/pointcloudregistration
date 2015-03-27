#include "ros/ros.h"
#include <image_transport/image_transport.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <image_transport/subscriber_filter.h>
#include "boost/thread.hpp"
#include "pointcloudregistration/pcl_registration.h"
#include "pointcloudregistration/pcl_analyser.h"
#include "pointcloudregistration/vision.h"
#include "pointcloudregistration/settings.h"
#include <pcl/visualization/pcl_visualizer.h>
PCL_registration* registrar=0;
PCL_analyser* pcl_analyse=0;
Vision* visor=0;
void frameCb(lsd_slam_viewer::keyframeMsgConstPtr msg)
{
	if(msg->time > lastFrameTime) return;

	if(registrar != 0)
		registrar->addFrameMsg(msg);
}
void graphCb(lsd_slam_viewer::keyframeGraphMsgConstPtr msg)
{
	if(registrar != 0)
		registrar->addGraphMsg(msg);
}
void callback(const sensor_msgs::ImageConstPtr& imgMsg ,const lsd_slam_viewer::keyframeMsg::ConstPtr &frameMsg)
{
	if(pcl_analyse==0 || visor==0)
		return;
	//check if both are ready
	if(!(visor->ready()) || !(pcl_analyse->ready()))
		return;
	//pass the data and do processing
	visor->process(imgMsg);
	//pcl_analyse->process(frameMsg);
}

void rosThreadLoop( int argc, char** argv )
{
	printf("Started ROS thread\n");

	ros::init(argc, argv, "registrar");
	ROS_INFO("registrar node starting");

	ros::NodeHandle nh;
	image_transport::ImageTransport it(nh);
	ros::Subscriber keyFrames_sub = nh.subscribe(nh.resolveName("lsd_slam/keyframes"),20, frameCb);
	ros::Subscriber graph_sub       = nh.subscribe(nh.resolveName("lsd_slam/graph"),10, graphCb);
	image_transport::SubscriberFilter image_sub(it, nh.resolveName("/ardrone/image_raw"),1);
	message_filters::Subscriber<lsd_slam_viewer::keyframeMsg> liveFrames_sub(nh,nh.resolveName("lsd_slam/liveframes"), 1);
	typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, lsd_slam_viewer::keyframeMsg> MySyncPolicy;
	// ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
	message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), image_sub, liveFrames_sub);
	sync.registerCallback(boost::bind(&callback, _1, _2));
	ROS_INFO("subscribers initialized - going for a spin");

	ros::spin();

	ros::shutdown();

	printf("Exiting ROS thread\n");

	exit(1);
}


int main( int argc, char** argv )
{
	boost::thread rosThread;
	// start ROS thread
	rosThread = boost::thread(rosThreadLoop, argc, argv);
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("cloud"));
	int v1(0);
	viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
  	viewer->setBackgroundColor (0, 0, 0,v1);

	registrar = new PCL_registration();
	visor = new Vision();
	pcl_analyse = new PCL_analyser();
	PointCloud::Ptr tmp=registrar->getPCL();
  	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(tmp);
	viewer->addPointCloud(tmp,rgb,"cloud",v1);
	int v2(0);
	viewer->createViewPort(0.5, 0.0, 1.0, 1.0, v2);
	viewer->setBackgroundColor (0.3, 0.3, 0.3, v2);
  	//pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgba(registrar->getDepth());
	//viewer->addPointCloud(registrar->getDepth(),rgba,"depth",v2);
	viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1,"cloud");
	//viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1,"depth");
	viewer->initCameraParameters ();
	viewer->addCoordinateSystem (1.0);
while (!viewer->wasStopped ())
  {
	if(registrar->PCLUpdate())
		viewer->updatePointCloud(registrar->getPCL(),"cloud");
	//viewer->updatePointCloud(registrar->getDepth(),"depth");
    viewer->spinOnce (100);
    boost::this_thread::sleep (boost::posix_time::microseconds (100000));
  }
	printf("Shutting down... \n");
	ros::shutdown();
	rosThread.join();
	printf("Done. \n");

}
