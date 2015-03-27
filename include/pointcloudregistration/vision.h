#ifndef VISION_H
#define VISION_H
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <boost/thread.hpp>

class Vision
{
    public:
        Vision();
        virtual ~Vision();
	void process(const sensor_msgs::ImageConstPtr&);
	bool ready();
    protected:
    private:
	ros::NodeHandle nh;
	image_transport::ImageTransport it;
	image_transport::Publisher image_lsd;
        cv::Ptr<cv::LineSegmentDetector> ls;
	cv_bridge::CvImagePtr cv_input_ptr,cv_lsd_ptr;
	bool wantExit,data_ready;
	cv::Mat InputGray;
        std::vector<cv::Vec4i> lines_std;
	boost::mutex imageMutex;
	boost::thread worker;
	void process();
	void getLines();
	boost::condition_variable newData;
	void threadLoop();
};

#endif // PCL_registration_H
