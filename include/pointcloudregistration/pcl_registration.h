#ifndef PCL_REGISTRATION_H
#define PCL_REGISTRATION_H
#include <ros/ros.h>
#include "pcl_ros/point_cloud.h"
#include "pcl/point_types.h"
#include "sophus/sim3.hpp"
#include "pointcloudregistration/KeyFrameGraph.h"
#include "pointcloudregistration/KeyFrame.h"
#include "pointcloudregistration/datastructures.h"
#include <map>


class PCL_registration
{
    public:
        PCL_registration(KeyFrameGraph*);
        virtual ~PCL_registration();
        void addFrameMsg(lsd_slam_viewer::keyframeMsgConstPtr);
	void addGraphMsg(lsd_slam_viewer::keyframeGraphMsgConstPtr);
	void drawPlane(pcl::PointCloud<pcl::PointXYZ>::ConstPtr,Eigen::Affine3f & );

    protected:
    private:
	boost::mutex meddleMutex,planeMutex;
	KeyFrameGraph* graph;
	std::map<int, PointCloud::Ptr> cloudsByID;
	pcl::PointCloud<pcl::PointXYZ>::Ptr planeCloud;
	boost::thread visualiser;
	void visualiserThread();
	bool wantExit,newPlane;
	Eigen::Affine3f campose;
};

#endif // PCL_registration_H
