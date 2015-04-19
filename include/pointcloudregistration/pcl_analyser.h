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
#include "pointcloudregistration/datastructures.h"
#include <cv_bridge/cv_bridge.h>
class KeyFrameGraph;
class LineReg;
class PCL_analyser
{
    public:
        PCL_analyser(KeyFrameGraph*,LineReg*);
        virtual ~PCL_analyser();
	void process(lsd_slam_viewer::keyframeMsgConstPtr);
	bool ready();
    protected:
    private:
    	PointCloud::Ptr cloud, depth;
	ros::NodeHandle nh;
	KeyFrameGraph* graph;
	LineReg* stairs;
	image_transport::ImageTransport it;
	image_transport::Publisher pub_depth,pub_depthf,pub_curv,pub_K,pub_H;
	boost::condition_variable newData,openCVdisplaySignal;
	boost::thread worker;
	boost::mutex frameMutex,cloudMtx;
	Sophus::Sim3f camToWorld;
	void calcCurvature();
	int width, height;
	cv_bridge::CvImagePtr cv_depth_ptr, cv_depthf_ptr,cv_H_ptr,cv_K_ptr,cv_CI_ptr;
	bool wantExit,data_ready;
	//camera parameters
	float fx,fy,cx,cy;
	void getDepthImage();
	void threadLoop();
	void filterDepth();
	void writeHist(float,float,int,cv::UMat);
	std::vector<KeyFrame*> keyframes;
	std_msgs::Header header;
	Eigen::Matrix4f soph;
	int my_scaleDepthImage;
	cv::Mat depthImg;
	cv::UMat filt;
	sensor_msgs::ImagePtr msg;
};

#endif // PCL_ANALYSER_H
