#include "pointcloudregistration/pcl_registration.h"
#include <boost/lexical_cast.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <iostream>
PCL_registration::PCL_registration(KeyFrameGraph* keyGraph):  graph(keyGraph), nh("~"), wantExit(false)
{
	pub = nh.advertise<sensor_msgs::PointCloud2> ("points2", 1);
	visualiser = boost::thread(boost::bind(&PCL_registration::visualiserThread,this));
	ROS_INFO("registration ready");
     //ctor
}

PCL_registration::~PCL_registration()
{
	std::cout<<"called pcl_registration destructor"<< std::endl;
	wantExit=true;
	ros::shutdown();
	std::cout<<"waiting for thread to close"<< std::endl;
	visualiser.join();
	for(std::map<int, PointCloud::Ptr>::iterator it= cloudsByID.begin(); it != cloudsByID.end(); it++) {
		it->second.reset();
	}
	delete graph;
    //dtor
}

void PCL_registration::addFrameMsg(lsd_slam_viewer::keyframeMsgConstPtr msg)
{
	meddleMutex.lock();
		sensor_msgs::PointCloud2::Ptr points = graph->addMsg(msg);
		pub.publish(points);
	meddleMutex.unlock();
}
void PCL_registration::addGraphMsg(lsd_slam_viewer::keyframeGraphMsgConstPtr msg)
{
	meddleMutex.lock();
	graph->addGraphMsg(msg);
	meddleMutex.unlock();
}

void PCL_registration::visualiserThread()
{
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("cloud"));
  	viewer->setBackgroundColor (0, 0, 0);
	viewer->initCameraParameters ();
	viewer->addCoordinateSystem (1.0);
	Eigen::Affine3f trans;
	std::vector<KeyFrame*> keyframes;
	ROS_INFO("started viewer");
while (!wantExit)
  {
	keyframes=graph->getFrames();

	if(graph->PCLUpdate())
	{ 
		for(std::size_t i=0;i<keyframes.size();i++)
		{
			trans=keyframes[i]->camToWorld.matrix();
			std::string id = boost::lexical_cast<std::string>(keyframes[i]->id);
			if(!viewer->updatePointCloudPose(id,trans))
			{
			//add cloud if id not recognized
			PointCloud::Ptr tmp(new PointCloud);
			*tmp=*(keyframes[i]->getPCL());
			cloudsByID[keyframes[i]->id]=tmp;
	  		pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloudsByID[keyframes[i]->id]);
			viewer->addPointCloud(cloudsByID[keyframes[i]->id],rgb,id);
			viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1,id);
			//update pose
			if(!viewer->updatePointCloudPose(id,trans))
				ROS_WARN_STREAM("no pointcloud with id: " << id);
			keyframes[i]->release();
			}
		}
	}
	viewer->spinOnce (100);
	boost::this_thread::sleep (boost::posix_time::microseconds (100000));
  }
	viewer->removeAllPointClouds();
	viewer->spinOnce();
	viewer->close();
	viewer.reset();
}
