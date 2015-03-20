#include "pointcloudregistration/pcl_registration.h"
#include <boost/lexical_cast.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>


PCL_registration::PCL_registration(): cloud (new PointCloud)
{
	nh=ros::NodeHandle("~");
	pub = nh.advertise<sensor_msgs::PointCloud2> ("points2", 1);
     ROS_INFO("registrar constructed");
	graph= (new KeyFrameGraph);
     //ctor
}

PCL_registration::~PCL_registration()
{
	delete graph;
    //dtor
}
void PCL_registration::getDepthImage(lsd_slam_viewer::keyframeMsgConstPtr msg)
{
	Sophus::Sim3f camToWorld;
	memcpy(camToWorld.data(), msg->camToWorld.data(), 7*sizeof(float));
	Sophus::Sim3f worldToCam=camToWorld.inverse();
	Eigen::Matrix4f transformation = worldToCam.matrix();
	PointCloud::Ptr transformed (new PointCloud);
	pcl::transformPointCloud(*cloud, *transformed, transformation);

}

void PCL_registration::addFrameMsg(lsd_slam_viewer::keyframeMsgConstPtr msg)
{
	meddleMutex.lock();
	if(!msg->isKeyframe)
	{
		/*if(currentCamDisplay->id > msg->id)
		{
			printf("detected backward-jump in id (%d to %d), resetting!\n",);// currentCamDisplay->id, msg->id);
		}/*
		currentCamDisplay->setFrom(msg);
		lastAnimTime = lastCamTime = msg->time;
		lastCamID = msg->id;*/
	}
	else{
		sensor_msgs::PointCloud2::Ptr points = graph->addMsg(msg);
		pub.publish(points);
		graph->refreshPCL();
	}
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
	ROS_INFO("retreiving cloud");
	return cloud;
}
bool PCL_registration::PCLUpdate()
{
	return graph->PCLUpdate();
}
