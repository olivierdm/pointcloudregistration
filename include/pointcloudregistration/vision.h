#ifndef VISION_H
#define VISION_H
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <boost/thread.hpp>
#include <tum_ardrone/filter_state.h>
class LineReg;
class Vision
{
    public:
        Vision(LineReg*);
        virtual ~Vision();
	void process(const sensor_msgs::ImageConstPtr&,const tum_ardrone::filter_stateConstPtr&);
	bool ready();
    protected:
    private:
	std::string stair_cascade_name;
	cv::CascadeClassifier stair_cascade;
	LineReg* stairs;
	ros::NodeHandle nh;
	image_transport::ImageTransport it;
	image_transport::Publisher pub_lsd, pub_detect;
	cv::Ptr<cv::LineSegmentDetector> ls;
	cv_bridge::CvImagePtr cv_input_ptr,cv_lsd_ptr,cv_det_ptr;
	bool wantExit,data_ready;
	std::vector<cv::Rect> rectangles;
	cv::Mat InputGray;
	//use std::vector<cv::Vec4i> for older implementations
        std::vector<cv::Vec4f> lines;
	std::vector<float> quality;
	boost::mutex imageMutex;
	boost::thread worker;
	void getLines();
	void detect();
	boost::condition_variable newData;
	void threadLoop();
	tum_ardrone::filter_stateConstPtr pose;
};

#endif // VISION_H
