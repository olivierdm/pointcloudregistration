#ifndef PCL_ANALYSER_H
#define PCL_ANALYSER_H
#include <ros/ros.h>
#include <std_msgs/Header.h>
#include "lsd_slam_viewer/keyframeMsg.h"
#include <image_transport/image_transport.h>
#include <opencv2/core/core.hpp>
#include "sophus/sim3.hpp"
#include <Eigen/Core>
#include "pcl_ros/point_cloud.h"
#include "pcl/point_types.h"
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
	PointCloud::Ptr getCloud();
	void release();
    protected:
    private:
	ros::NodeHandle nh;
	image_transport::ImageTransport it;
	image_transport::Publisher pub;
    	PointCloud::Ptr cloud,depth, boundingbox;
	boost::condition_variable newData,openCVdisplaySignal;
	boost::thread worker;
	boost::mutex frameMutex,cloudMtx;
	Sophus::Sim3f camToWorld;
	void calcCurvature();
	int width, height;
	bool wantExit,data_ready;
	//camera parameters
	float fx,fy,cx,cy;
	void getDepthImage();
	void threadLoop();
	std::vector<KeyFrame*> keyframes;
	std_msgs::Header header;
	KeyFrameGraph* graph;
	Eigen::Matrix4f soph;
	int my_scaleDepthImage;
	cv::Mat depthImg;
	sensor_msgs::ImagePtr msg;
};

#endif // PCL_ANALYSER_H
