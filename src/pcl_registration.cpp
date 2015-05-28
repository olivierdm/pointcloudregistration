#include "pointcloudregistration/pcl_registration.h"
#include "pointcloudregistration/KeyFrameGraph.h"
#include "pointcloudregistration/KeyFrame.h"
#include <boost/lexical_cast.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <iostream>
PCL_registration::PCL_registration(std::shared_ptr<KeyFrameGraph>& keyGraph):  graph(keyGraph), planeCloud(new pcl::PointCloud<pcl::PointXYZ>), wantExit(false),newPlane(false), resetRequested(false), lastid(0)
{
/// \brief default constructor, initialize visualiser thread
	visualiser = boost::thread(boost::bind(&PCL_registration::visualiserThread,this));
	ROS_INFO("registration ready");
     //ctor
}

PCL_registration::~PCL_registration()
{
/// \brief default destructor, shuts down the visualiser thread.
	std::cout<<"called pcl_registration destructor"<< std::endl;
	wantExit=true;
	ros::shutdown();
	std::cout<<"waiting for thread to close"<< std::endl;
	visualiser.join();
	eraseClouds();
    //dtor
}

bool PCL_registration::addFrameMsg(lsd_slam_viewer::keyframeMsgConstPtr msg)
{
/// \brief Add Keyframe or check if lsdslam is ok with liveframe.
/// \return Returns true if jump detected.
	boost::mutex::scoped_lock lock(meddleMutex);
	bool ret(false);
	if(!msg->isKeyframe)
	{
		if(lastid > msg->id)
		{
			ROS_WARN_STREAM("detected backward-jump in id ("<< lastid << " to " << msg->id << "), resetting!");
			resetRequested = true;
			ret= true;
		}
		lastid = msg->id;
	}else
	graph->addMsg(msg);
return ret;
}
void PCL_registration::addGraphMsg(lsd_slam_viewer::keyframeGraphMsgConstPtr msg)
{
/// \brief Pass the graph message to the keyframe graph.
	boost::mutex::scoped_lock lock(meddleMutex);
	graph->addGraphMsg(msg);
}

void PCL_registration::visualiserThread()
{
/// \brief function that initialises viewer and shows updates
	std::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("cloud"));
  	viewer->setBackgroundColor (0, 0, 0);
	viewer->initCameraParameters ();
	viewer->addCoordinateSystem (1.0);
	Eigen::Affine3f trans;
	std::vector<std::shared_ptr<KeyFrame>> keyframes;
	ROS_INFO("started viewer");
while (!wantExit)
  {
	if(resetRequested){
		ROS_DEBUG("viewer reset requested");
		viewer->removeAllPointClouds();
		eraseClouds();
		graph->reset();
		resetRequested=false;
	}
	keyframes=graph->getFrames();
	if(graph->PCLUpdate())
	{ 
		for(std::size_t i=0;i<keyframes.size();i++)
		{
			trans=keyframes[i]->camToWorld.matrix();
			std::string id = boost::lexical_cast<std::string>(keyframes[i]->id);
		ROS_DEBUG_STREAM("cloud: "<< keyframes[i]->id);
			if(!viewer->updatePointCloudPose(id,trans))
			{
			//add cloud if id not recognized
			cloudsByID[keyframes[i]->id]=keyframes[i]->getPCL();
	  		//pcl::visualization::PointCloudColorHandlerRGBField<Point> rgb(cloudsByID[keyframes[i]->id]);
			viewer->addPointCloud(cloudsByID[keyframes[i]->id]/*,rgb*/,id);
			ROS_DEBUG_STREAM("adding keyframe");
			viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1,id);
			//update pose
			if(!viewer->updatePointCloudPose(id,trans))
				ROS_WARN_STREAM("no pointcloud with id: " << id);
			}
		}
	}
	if(newPlane)
	{
		boost::mutex::scoped_lock lock(planeMutex);
		newPlane = false;
		viewer->removePointCloud("stair");
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> stair_color (planeCloud, 230, 20, 20); // Red
		viewer->addPointCloud (planeCloud, stair_color, "stair");
		viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2,"stair");
		viewer->updatePointCloudPose("stair",campose);
	}
	viewer->spinOnce (100);
	boost::this_thread::sleep (boost::posix_time::microseconds (50000));
  }
	viewer->removeAllPointClouds();
	viewer->spinOnce();
	viewer->close();
}
void PCL_registration::eraseClouds()
{
/// \brief erase clouds in the viewer
	for(auto it= cloudsByID.begin(); it != cloudsByID.end(); it++) {
		it->second.reset();
	}
}
void PCL_registration::drawPlane(const pcl::PointCloud<pcl::PointXYZ>::Ptr& plane, const Eigen::Affine3f & pose)
{
/// \brief Add plane with plane inliers within the EKF pose.
	boost::mutex::scoped_lock lock(planeMutex);
	planeCloud=plane;
	campose=pose;
	newPlane =true;
}
