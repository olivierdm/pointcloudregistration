#ifndef LINEREG_H
#define LINEREG_H
#include <ros/ros.h>
#include <boost/thread.hpp>
#include <opencv2/opencv.hpp>
class LineReg
{
    public:
        LineReg();
        virtual ~LineReg();
	void process(cv::UMat,cv::UMat);
	void process(std::vector<cv::Rect>,std::vector<cv::Vec4f> lines, std::vector<float> quality);
	bool ready();
    protected:
    private:
	ros::NodeHandle nh;
	//image_transport::ImageTransport it;
	//image_transport::Publisher pub_lsd, pub_detect;
	//cv_bridge::CvImagePtr cv_input_ptr,cv_lsd_ptr;
	bool wantExit,rectangles_ready,curvature_ready,lines_ready;
        std::vector<cv::Vec4f> lines_std;
	boost::mutex dataMutex;
	boost::thread worker;
	void get3DLines();
	boost::condition_variable newData;
	void threadLoop();
};

#endif // LINEREG_H
