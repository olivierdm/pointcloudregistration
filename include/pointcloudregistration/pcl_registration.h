#ifndef PCL_REGISTRATION_H
#define PCL_REGISTRATION_H
#include <ros/ros.h>
#include "pcl_ros/point_cloud.h"
#include "pcl/point_types.h"
#include <pcl/filters/median_filter.h>
#include "sophus/sim3.hpp"
#include "pointcloudregistration/KeyFrameGraph.h"
#include "pointcloudregistration/KeyFrame.h"
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;

class PCL_registration
{
    public:
        PCL_registration();
        virtual ~PCL_registration();
        void addFrameMsg(lsd_slam_viewer::keyframeMsgConstPtr);
	void addGraphMsg(lsd_slam_viewer::keyframeGraphMsgConstPtr);
	PointCloud::Ptr getPCL();
	bool PCLUpdate();
	void getDepthImage(lsd_slam_viewer::keyframeMsgConstPtr);
    protected:
    private:
	ros::NodeHandle nh;
	ros::Publisher pub;
    	PointCloud::Ptr cloud;
	boost::mutex meddleMutex;
	KeyFrameGraph* graph;
	int currentCamID;


};

#endif // PCL_registration_H
