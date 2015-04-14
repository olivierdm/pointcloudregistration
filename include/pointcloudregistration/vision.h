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
class Vision
{
    public:
        Vision();
        virtual ~Vision();
	void process(const sensor_msgs::ImageConstPtr&,const tum_ardrone::filter_stateConstPtr&);
	bool ready();
    protected:
    private:
	std::string stair_cascade_name;
	cv::CascadeClassifier stair_cascade;
	ros::NodeHandle nh;
	image_transport::ImageTransport it;
	image_transport::Publisher image_lsd;
	image_transport::Publisher image_detect;
        cv::Ptr<cv::LineSegmentDetector> ls;
	cv_bridge::CvImagePtr cv_input_ptr,cv_lsd_ptr;
	bool wantExit,data_ready;
	cv::Mat InputGray;
	//use std::vector<cv::Vec4i> for older implementations
        std::vector<cv::Vec4f> lines_std;
	boost::mutex imageMutex;
	boost::thread worker;
	void process();
	void getLines();
	void detect();
	boost::condition_variable newData;
	void threadLoop();
	tum_ardrone::filter_stateConstPtr pose;
};

#endif // PCL_registration_H
