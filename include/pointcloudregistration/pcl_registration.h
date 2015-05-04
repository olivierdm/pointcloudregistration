#ifndef PCL_REGISTRATION_H
#define PCL_REGISTRATION_H
#include <ros/ros.h>
#include "pcl_ros/point_cloud.h"
#include "pcl/point_types.h"
#include "sophus/sim3.hpp"
#include "lsd_slam_viewer/keyframeGraphMsg.h"
#include "lsd_slam_viewer/keyframeMsg.h"
#include "boost/thread.hpp"
#include "pointcloudregistration/datastructures.h"
#include <map>
class KeyFrameGraph;

class PCL_registration
{
	public:
		PCL_registration(std::shared_ptr<KeyFrameGraph>&);
		virtual ~PCL_registration();
		bool addFrameMsg(lsd_slam_viewer::keyframeMsgConstPtr);
		void addGraphMsg(lsd_slam_viewer::keyframeGraphMsgConstPtr);
		void drawPlane(const pcl::PointCloud<pcl::PointXYZ>::Ptr&,const Eigen::Affine3f & );

    protected:
    private:
	boost::mutex meddleMutex,planeMutex;
	std::shared_ptr<KeyFrameGraph> graph;
	std::map<int, PointCloud::Ptr> cloudsByID;
	pcl::PointCloud<pcl::PointXYZ>::ConstPtr planeCloud;
	boost::thread visualiser;
	void visualiserThread();
	void eraseClouds();
	bool wantExit, newPlane, resetRequested;
	int lastid;
	Eigen::Affine3f campose;
};

#endif // PCL_registration_H
