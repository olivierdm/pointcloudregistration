#ifndef VISION_H
#define VISION_H
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/objdetect/objdetect.hpp>
//#include <tum_ardrone/filter_state.h>
class Vision
{
    public:
        Vision();
        virtual ~Vision();
	void operator()(const cv_bridge::CvImagePtr&, std::vector<cv::Rect>&, std::vector<cv::Vec4f>&);
    protected:
    private:
	std::string stair_cascade_name;
	tf::TransformListener listener;
	cv::CascadeClassifier stair_cascade;
	ros::NodeHandle nh;
	image_transport::ImageTransport it;
	image_transport::Publisher pub_lsd, pub_detect;
	cv::Ptr<cv::LineSegmentDetector> ls;
	//use std::vector<cv::Vec4i> for older implementations
	void getLines(const cv_bridge::CvImagePtr &, cv::Mat&, std::vector<cv::Vec4f>&);
	void detect(const cv_bridge::CvImagePtr &, cv::Mat&, std::vector<cv::Rect>&);
};

#endif // VISION_H
