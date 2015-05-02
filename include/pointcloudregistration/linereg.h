#ifndef LINEREG_H
#define LINEREG_H
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <pcl/common/common.h>
#include <tum_ardrone/filter_state.h>
#include "lsd_slam_viewer/keyframeMsg.h"
struct DepthLine;
struct Candidate;
class LineReg
{
    public:
        LineReg();
        virtual ~LineReg();
	bool operator()(cv::UMat, cv::UMat&, cv::UMat&, std::vector<cv::Rect>&, std::vector<cv::Vec4f>&, cv_bridge::CvImagePtr &, const lsd_slam_viewer::keyframeMsgConstPtr&, const tum_ardrone::filter_stateConstPtr&);
	bool SegmentIntersectRectangle(cv::Rect&, cv::Vec4f&);
    protected:
    private:
	ros::NodeHandle nh;
	image_transport::ImageTransport it;
	image_transport::Publisher pub_det, pub_can;
	void getParallelLines(Candidate &, std::vector<DepthLine>&);
	void get3DLines(Candidate &, cv::Mat, cv::Mat, cv::Mat, float&, float&, float&, float&);
	void getPlane(Candidate &, cv::Mat&, cv::Mat&, const tum_ardrone::filter_stateConstPtr&);
};

#endif // LINEREG_H
