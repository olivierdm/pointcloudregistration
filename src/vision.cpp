#include "pointcloudregistration/vision.h"
#include "pointcloudregistration/settings.h"
#include <opencv2/opencv.hpp>
#include <math.h>       /* tan */
#include "ros/package.h"

Vision::Vision():nh("~"),it(nh)
{
/// \brief default constructor, sets up the cascade classifier
	pub_lsd = it.advertise("lsd",1);
	pub_detect = it.advertise("detect",1);
	ls = cv::createLineSegmentDetector(cv::LSD_REFINE_ADV);
	stair_cascade_name = ros::package::getPath("pointcloudregistration")+"/src/stair_cascade.xml";
	if( !stair_cascade.load( stair_cascade_name ) )
		ROS_WARN_STREAM("Error loading cascade: " << stair_cascade_name);
}
Vision::~Vision()
{
	std::cout<<"visor destroyed"<< std::endl;
	//dtor
}
void Vision::operator()(const cv_bridge::CvImagePtr& cv_input_ptr, std::vector<cv::Rect>& rectangles, std::vector<cv::Vec4f>& lines)
{
///
/// \brief  Accepts the data of an image and copies the neccesary data to the adequate private variables.
/// @param[in] cv_input_ptr cv_bridge image coming from the front camera
/// @param[out] rectangles the zones detected by the object detector
/// @param[out] lines vector containing the lines
/// 

	ROS_INFO("process image");
	cv::Mat inputGray;
/// Convert input to grayscale
	cv::cvtColor(cv_input_ptr->image, inputGray, cv::COLOR_BGR2GRAY);
/// Detect lines
	getLines(cv_input_ptr, inputGray, lines);
/// Perform object detection
	detect(cv_input_ptr, inputGray, rectangles);
}

void Vision::getLines(const cv_bridge::CvImagePtr& cv_input_ptr, cv::Mat& inputGray, std::vector<cv::Vec4f>& lines)
{
///
/// \brief Detects lines using the line segment detector implemented in OpenCV.
/// @param[in] cv_input_ptr cv_bridge image coming from the front camera
/// @param[in] inputGray grayscale version of the input image
/// @param[out] lines vector containing the lines

	std::vector<cv::Vec4f> bad_lines;
	std::vector<float> quality_std;
/// Detect using lsd.
	ls->detect(inputGray, lines,cv::noArray(),cv::noArray(),quality_std);
/// Get transform
	tf::StampedTransform transform;
	try{
		listener.lookupTransform("/map", "/tum_base_frontcam", cv_input_ptr->header.stamp, transform);
	}
	catch (tf::TransformException ex){
		ROS_ERROR("%s",ex.what());
		ros::Duration(1.0).sleep();
		return;
	}

	///  acess roll pitch and yaw
	double roll, pitch, yaw;
	transform.getBasis().getRPY(roll, pitch, yaw);
	float maxtan = std::tan(static_cast<float>(roll-CV_PI/2.0+maxAngle/180.0 * CV_PI));
	float mintan = std::tan(static_cast<float>(roll-CV_PI/2.0-maxAngle/180.0 * CV_PI));
	auto iter(remove_if(lines.begin(),lines.end(),[&maxtan,&mintan](cv::Vec4f& line) {
		float mytan=(line[3] - line[1])/(line[2]-line[0]);
		return mytan<mintan || mytan>maxtan;}));
	bad_lines.insert(bad_lines.begin(),iter,lines.end());
	lines.erase(iter,lines.end());
/// Draw lines on the output image.
	if (pub_lsd.getNumSubscribers() != 0)
	{
		cv_bridge::CvImage cv_lsd(cv_input_ptr->header,"bgr8",cv_input_ptr->image.clone());

		float y =(cv_input_ptr->image.cols)/2.0f*tan(roll-CV_PI/2.0);
		float y1= static_cast<float> (cv_lsd.image.rows) / 2.0f + y;
		float y2= static_cast<float> (cv_lsd.image.rows) / 2.0f - y;
		ls->drawSegments(cv_lsd.image,bad_lines);
		cv::line(cv_lsd.image,cv::Point2f(0.0f,y1),cv::Point2f(static_cast<float>(cv_lsd.image.cols),y2),cv::Scalar(255,0,0));
		for(size_t i=0 ; i < lines.size();i++)
		{
			cv::Scalar color(0, quality_std[i]*255,0);
			cv::line(cv_lsd.image,cv::Point2f(lines[i][0],lines[i][1]),cv::Point2f(lines[i][2],lines[i][3]),color);
		}
		pub_lsd.publish(cv_lsd.toImageMsg());
	}
}

void Vision::detect(const cv_bridge::CvImagePtr& cv_input_ptr, cv::Mat& inputGray, std::vector<cv::Rect>& rectangles)
{
///
/// \brief Detect stairs using a cascade classifier.
/// @param[in] cv_input_ptr cv_bridge image coming from the front camera
/// @param[in] inputGray grayscale version of the input image
/// @param[out] rectangles the zones detected by the object detector
	cv::equalizeHist( inputGray, inputGray );
/// Detect rectangles.
	stair_cascade.detectMultiScale(  inputGray , rectangles, 1.1, 1, 0|CV_HAAR_SCALE_IMAGE, cv::Size(90,150 ) );
/// Draw rectangles on the output image.
	if (pub_detect.getNumSubscribers() != 0)
	{
	cv_bridge::CvImage cv_det(cv_input_ptr->header,"bgr8",cv_input_ptr->image.clone());
	for( size_t i = 0; i < rectangles.size(); i++ )
	{
		cv::rectangle(cv_det.image, rectangles[i], cv::Scalar( 255, 0, 255 ), 4, 8, 0);
	}
	pub_detect.publish(cv_det.toImageMsg());
	}
}
