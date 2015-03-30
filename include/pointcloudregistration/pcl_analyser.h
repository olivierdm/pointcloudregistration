#ifndef PCL_ANALYSER_H
#define PCL_ANALYSER_H
#include <ros/ros.h>
#include <std_msgs/Header.h>
#include "lsd_slam_viewer/keyframeMsg.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <Eigen/Core>
#include "pcl_ros/point_cloud.h"
#include "pcl/point_types.h"
#include <pcl/filters/median_filter.h>
#include "sophus/sim3.hpp"
#include "opencv2/highgui.hpp"
#include <boost/thread.hpp>
#include "pointcloudregistration/pcl_registration.h"

typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;

class PCL_analyser
{
    public:
        PCL_analyser(KeyFrameGraph*);
        virtual ~PCL_analyser();
	void process(lsd_slam_viewer::keyframeMsgConstPtr);
	bool ready();


    protected:
    private:
	ros::NodeHandle nh;
	image_transport::ImageTransport it;
	image_transport::Publisher pub;
    	PointCloud::Ptr cloud,depth, boundingbox;
	boost::condition_variable newData;
	boost::thread worker;
	boost::mutex frameMutex,cloudMutex,framesMtx;
	Sophus::Sim3f camToWorld;
	void calcBox(lsd_slam_viewer::keyframeMsgConstPtr);
	void calcCurvature();
	float minZ,maxZ;
	int width, height;
	bool wantExit,data_ready;
	Eigen::Matrix3f camera;
	Eigen::Matrix<float,3,4> rotTrans,projection;
	void setProjection(Sophus::Sim3f);
	void setCamera(float,float,float,float);
	void getDepthImage();
	void threadLoop();
	std::vector<KeyFrame*> keyframes;
	std_msgs::Header header;
	KeyFrameGraph* graph;
};

#endif // PCL_ANALYSER_H
