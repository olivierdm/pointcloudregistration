#ifndef PCL_ANALYSER_H
#define PCL_ANALYSER_H
#include <ros/ros.h>
#include "lsd_slam_viewer/keyframeMsg.h"
#include <image_transport/image_transport.h>
#include <opencv2/core/core.hpp>

class KeyFrame;
struct framedist;
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
	void calcCurvature(const cv::UMat&, const float&, const float&, const int&, cv::UMat & , cv::UMat&, cv::UMat & );
	void getDepthImage(std::vector<framedist>&, const float&, const float&, const float&, const float&, cv::Mat&);
	void filterDepth(const cv::Mat&, const int&, cv::UMat&);
	void writeHist(float,float,int,cv::UMat);
};

#endif // PCL_ANALYSER_H
