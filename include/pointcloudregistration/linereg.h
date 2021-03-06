#ifndef LINEREG_H
#define LINEREG_H
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PointStamped.h>
#include <Eigen/Geometry>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <pcl/common/common.h>
//#include <tum_ardrone/filter_state.h>
#include "lsd_slam_viewer/keyframeMsg.h"
#include <fstream>
#include <boost/filesystem.hpp>
struct DepthLine;
struct Candidate;
class LineReg
{
    public:
        LineReg();
        virtual ~LineReg();
	bool operator()(cv::UMat, cv::UMat, cv::UMat, std::vector<cv::Rect>, std::vector<cv::Vec4f>, cv_bridge::CvImagePtr, const lsd_slam_viewer::keyframeMsgConstPtr, pcl::PointCloud<pcl::PointXYZ>::Ptr&, Eigen::Affine3f&);
	bool SegmentIntersectRectangle(cv::Rect&, cv::Vec4f&);
    protected:
    private:
	ros::NodeHandle nh;
	image_transport::ImageTransport it;
	image_transport::Publisher pub_det, pub_can;
	ros::Publisher point_pub;
	unsigned int frame_id, can_id;
	std::ofstream perfres;
	boost::filesystem::path dir;
	tf::TransformListener listener;
	void getParallelLines(Candidate &, std::vector<DepthLine>&);
	void get3DLines(Candidate &, const cv::Mat&, const cv::Mat&, const cv::Mat&, float&, float&, float&, float&);
	void getPlane(Candidate &, cv::Mat&, cv::Mat&, const Eigen::Affine3d&);
};

#endif // LINEREG_H
