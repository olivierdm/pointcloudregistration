#ifndef VISION_H
#define VISION_H
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <boost/thread.hpp>
#include <tum_ardrone/filter_state.h>
class Vision
{
    public:
        Vision();
        virtual ~Vision();
	//void process(const sensor_msgs::ImageConstPtr&,const tum_ardrone::filter_stateConstPtr&);
	//bool ready();
	void operator()(cv_bridge::CvImagePtr&, const tum_ardrone::filter_stateConstPtr&, std::vector<cv::Rect>&, std::vector<cv::Vec4f>&);
    protected:
    private:
	std::string stair_cascade_name;
	cv::CascadeClassifier stair_cascade;
	ros::NodeHandle nh;
	image_transport::ImageTransport it;
	image_transport::Publisher pub_lsd, pub_detect;
	cv::Ptr<cv::LineSegmentDetector> ls;
	//use std::vector<cv::Vec4i> for older implementations
	std::vector<float> quality;
	boost::mutex imageMutex;
	//boost::thread worker;
	void getLines(const cv_bridge::CvImagePtr &, cv::Mat&, const tum_ardrone::filter_stateConstPtr&, std::vector<cv::Vec4f>&);
	void detect(const cv_bridge::CvImagePtr &, cv::Mat&, std::vector<cv::Rect>&);
	//void threadLoop();
};

#endif // VISION_H
