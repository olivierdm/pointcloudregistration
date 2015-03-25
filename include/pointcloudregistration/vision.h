#ifndef VISION_H
#define VISION_H
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <boost/mutex.h>

class Vision
{
    public:
        Vision();
        virtual ~Vision();
        void addImgMsg(const sensor_msgs::ImageConstPtr&);
	void process();
    protected:
    private:
	ros::NodeHandle nh;
	image_transport::ImageTransport it;
	ros::Publisher pub;
	sensor_msgs::ImageConstPtr imgMsg;
	boost::Mutex imgPtr;
};

#endif // PCL_registration_H
