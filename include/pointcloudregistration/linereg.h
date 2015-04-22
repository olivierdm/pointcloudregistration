#ifndef LINEREG_H
#define LINEREG_H
#include <ros/ros.h>
#include <Eigen/Core>
#include <boost/thread.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <pcl/common/common.h>
class PCL_registration;
class LineReg
{
    public:
        LineReg(PCL_registration*);
        virtual ~LineReg();
	void process(cv::Mat,cv::Mat,cv::Mat,cv::Vec4f, const Eigen::Affine3f &);
	void process(std::vector<cv::Rect>,std::vector<cv::Vec4f> lines, std::vector<float> quality,cv::Mat);
	bool ready();
    protected:
    private:
	PCL_registration* registrar;
	ros::NodeHandle nh;
	image_transport::ImageTransport it;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
	image_transport::Publisher pub_det;
	cv_bridge::CvImagePtr cv_debug_ptr;
	bool wantExit,rectangles_ready,curvature_ready;
        std::vector<cv::Vec4f> lines;
	std::vector<float> lineQuality;
	std::vector<cv::Rect> rectangles;
	boost::mutex dataMutex;
	boost::thread worker;
	void get3DLines();
	boost::condition_variable newData;
	void threadLoop();
	cv::Mat depthImg,curv_weight,meanCurvature;
	float fxi,fyi,cxi,cyi;
	Eigen::Affine3f campose;
};

#endif // LINEREG_H