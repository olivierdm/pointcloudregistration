#ifndef LINEREG_H
#define LINEREG_H
#include <ros/ros.h>
#include <Eigen/Core>
#include <boost/thread.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <pcl/common/common.h>
#include <tum_ardrone/filter_state.h>
#include "lsd_slam_viewer/keyframeMsg.h"
class PCL_registration;
struct Candidate;
struct DepthLine;
class LineReg
{
    public:
        LineReg(PCL_registration*);
        virtual ~LineReg();
	void operator()(cv::UMat&, cv::UMat&, cv::UMat&, std::vector<cv::Rect>&, std::vector<cv::Vec4f>&, cv_bridge::CvImagePtr &, const lsd_slam_viewer::keyframeMsgConstPtr&, const tum_ardrone::filter_stateConstPtr&);
	bool SegmentIntersectRectangle(cv::Rect&, cv::Vec4f&);
    protected:
    private:
	PCL_registration* registrar;
	ros::NodeHandle nh;
	image_transport::ImageTransport it;
	image_transport::Publisher pub_det;
	std::vector<float> lineQuality;
	std::vector<DepthLine> depthLines;
	std::vector<Candidate> candidates;
	boost::mutex dataMutex;
	void getParallelLines(Candidate &);
	void get3DLines(Candidate &, cv::Mat, cv::Mat, cv::Mat, float&, float&, float&, float&);
	void getPlane(Candidate &, cv_bridge::CvImagePtr&, const tum_ardrone::filter_stateConstPtr&);
	Eigen::Affine3f campose;
};

#endif // LINEREG_H
