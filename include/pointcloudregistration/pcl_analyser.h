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
#include <cv_bridge/cv_bridge.h>
class KeyFrame;
class PCL_analyser
{
    public:
        PCL_analyser();
        virtual ~PCL_analyser();
	void operator()(lsd_slam_viewer::keyframeMsgConstPtr, std::vector<std::shared_ptr<KeyFrame>>&, cv::UMat&, cv::UMat&, cv::UMat&);
    private:
	ros::NodeHandle nh;
	image_transport::ImageTransport it;
	image_transport::Publisher pub_depth,pub_depthf,pub_curv,pub_K,pub_H;
	//boost::thread worker;
	Sophus::Sim3f camToWorld;
	void calcCurvature(const cv::UMat&, cv::UMat & ,cv::UMat & );
	int width, height;
	cv_bridge::CvImagePtr cv_depthf_ptr,cv_H_ptr,cv_K_ptr,cv_CI_ptr;
	//camera parameters
	float fx,fy,cx,cy;
	void getDepthImage(std::vector<std::shared_ptr <KeyFrame>>&, cv::Mat&);
	void filterDepth(cv::Mat &, cv::UMat &);
	void writeHist(float,float,int,cv::UMat);
	std::vector<KeyFrame*> keyframes;
	std_msgs::Header header;
	Eigen::Matrix4f soph;
	int my_scaleDepthImage;
	sensor_msgs::ImagePtr msg;
};

#endif // PCL_ANALYSER_H
