#include "pointcloudregistration/pcl_registration.h"
#include <boost/lexical_cast.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
PCL_registration::PCL_registration(): cloud (new PointCloud), depth(new PointCloud)
{
	nh=ros::NodeHandle("~");
	pub = nh.advertise<sensor_msgs::PointCloud2> ("points2", 1);
	graph= (new KeyFrameGraph);
	currentCamID=0;
	minX = maxX = minY = maxY=0.0;
	minZ= 0.3;
	maxZ=3;
	inter=10;
	ROS_INFO("registration ready");
     //ctor
}

PCL_registration::~PCL_registration()
{
	delete graph;
    //dtor
}

void PCL_registration::addFrameMsg(lsd_slam_viewer::keyframeMsgConstPtr msg)
{
	meddleMutex.lock();
		sensor_msgs::PointCloud2::Ptr points = graph->addMsg(msg);
		pub.publish(points);
		graph->refreshPCL();
	meddleMutex.unlock();
	cloud=graph->getPCL();
}
void PCL_registration::addGraphMsg(lsd_slam_viewer::keyframeGraphMsgConstPtr msg)
{
	meddleMutex.lock();
	graph->addGraphMsg(msg);
	meddleMutex.unlock();
}
PointCloud::Ptr PCL_registration::getPCL()
{
	ROS_INFO("retreiving cloud pointer");
	return cloud;
}

bool PCL_registration::PCLUpdate()
{
	return graph->PCLUpdate();
}
