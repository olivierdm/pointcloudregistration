#include "ros/ros.h"
#include "boost/thread.hpp"
#include "pointcloudregistration/pcl_registration.h"
#include "pointcloudregistration/settings.h"
#include <pcl/visualization/pcl_visualizer.h>
PCL_registration* registrar=0;
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


void rosThreadLoop( int argc, char** argv )
{
	printf("Started ROS thread\n");

	ros::init(argc, argv, "registrar");
	ROS_INFO("registrar node started");

	ros::NodeHandle nh;

	ros::Subscriber liveFrames_sub = nh.subscribe(nh.resolveName("lsd_slam/liveframes"),1, frameCb);
	ros::Subscriber keyFrames_sub = nh.subscribe(nh.resolveName("lsd_slam/keyframes"),20, frameCb);
	ros::Subscriber graph_sub       = nh.subscribe(nh.resolveName("lsd_slam/graph"),10, graphCb);

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
  	viewer->setBackgroundColor (0, 0, 0);
  	viewer->addCoordinateSystem (1.0);
	registrar = new PCL_registration();	
  	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(registrar->getPCL());
	viewer->addPointCloud(registrar->getPCL(),rgb,"cloud");
	viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1,"cloud");
	viewer->initCameraParameters ();
	
while (!viewer->wasStopped ())
  {
	if(registrar->PCLUpdate())
		viewer->updatePointCloud(registrar->getPCL(),"cloud");
    viewer->spinOnce (100);
    boost::this_thread::sleep (boost::posix_time::microseconds (100000));
  }
	printf("Shutting down... \n");
	ros::shutdown();
	rosThread.join();
	printf("Done. \n");

}
