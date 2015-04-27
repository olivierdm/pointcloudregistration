#include "pointcloudregistration/vision.h"
#include "pointcloudregistration/settings.h"
#include <opencv2/opencv.hpp>
#include <math.h>       /* tan */
#include "ros/package.h"


Vision::Vision():nh("~"),it(nh)
{
	pub_lsd = it.advertise("lsd",1);
	pub_detect = it.advertise("detect",1);
	ls = cv::createLineSegmentDetector(cv::LSD_REFINE_ADV);
	stair_cascade_name = ros::package::getPath("pointcloudregistration")+"/src/stair_cascade.xml";
	if( !stair_cascade.load( stair_cascade_name ) )
		{ROS_WARN_STREAM("Error loading cascade: " << stair_cascade_name);  };
	//initiate lsd image
	/*cv_lsd_ptr->encoding="bgr8";
	cv_lsd_ptr->header.frame_id="line_segment";
	cv_det_ptr->encoding="bgr8";*/
}
Vision::~Vision()
{
	std::cout<<"called vision destructor"<< std::endl;
	//destroy thread
	boost::mutex::scoped_lock lock(imageMutex);
	std::cout<<"visor destroyed"<< std::endl;
	//dtor
}
void Vision::operator()(cv_bridge::CvImagePtr & cv_input_ptr, const tum_ardrone::filter_stateConstPtr& poseMsg, std::vector<cv::Rect> & rectangles, std::vector<cv::Vec4f> & lines)
{
///
/// \brief  Accepts the data of an image and copies the neccesary data to the adequate private variables.
/// @param[in] msg an image
/// 

	ROS_INFO("process image");
	cv::Mat inputGray;
	cv::cvtColor(cv_input_ptr->image, inputGray, cv::COLOR_BGR2GRAY);
	getLines(cv_input_ptr, inputGray, poseMsg, lines);
	detect(cv_input_ptr, inputGray, rectangles);
}

void Vision::getLines(const cv_bridge::CvImagePtr & cv_input_ptr, cv::Mat & inputGray, const tum_ardrone::filter_stateConstPtr& pose, std::vector<cv::Vec4f> & lines)
{
///
/// \brief Detects lines using the line segment detector implemented in OpenCV.
///
	//cv_lsd_ptr->header =cv_input_ptr->header;
	//convert to grayscale
	//cv_lsd_ptr->image=cv_input_ptr->image.clone();

	std::vector<cv::Vec4f> bad_lines;
	std::vector<float> quality_std;
	//detect using lsd
	ls->detect(inputGray, lines,cv::noArray(),cv::noArray(),quality_std);
	float maxtan = std::tan(static_cast<float>(maxAngle)/180.0f * CV_PI);
	auto iter(remove_if(lines.begin(),lines.end(),[&maxtan](cv::Vec4f& line) {return std::abs((line[3] - line[1])/(line[2]-line[0]))>maxtan;} ));
	bad_lines.insert(bad_lines.begin(),iter,lines.end());
	lines.erase(iter,lines.end());

	if (pub_lsd.getNumSubscribers() != 0)
	{
		cv_bridge::CvImage cv_lsd(cv_input_ptr->header,"bgr8",cv_input_ptr->image.clone());
		float y =(cv_input_ptr->image.cols)/2.0f*tan(pose->roll/180.0*CV_PI);
		float y1= static_cast<float> (cv_lsd.image.rows) / 2.0f + y;
		float y2= static_cast<float> (cv_lsd.image.rows) / 2.0f - y;
		ls->drawSegments(cv_lsd.image,bad_lines);
		cv::line(cv_lsd.image,cv::Point2f(0.0f,y1),cv::Point2f(static_cast<float>(cv_lsd.image.cols),y2),cv::Scalar(255,0,0));
		cv::Scalar color(0,255,0);
		for(size_t i=0 ; i < lines.size();i++)
		{
			cv::line(cv_lsd.image,cv::Point2f(lines[i][0],lines[i][1]),cv::Point2f(lines[i][2],lines[i][3]),color);
		}
		pub_lsd.publish(cv_lsd.toImageMsg());
	}
}

void Vision::detect(const cv_bridge::CvImagePtr & cv_input_ptr, cv::Mat & inputGray, std::vector<cv::Rect> & rectangles)
{
///
/// \brief Detect stairs using a cascade classifier.
///
//cv::Mat rot =  cv::getRotationMatrix2D(Point2f(static_cast<float>(cv_input_ptr->image.rows)/2.0f, static_cast<float>(cv_input_ptr->image.cols)/2.0f),pose.roll, 1.0);
	cv::equalizeHist( inputGray, inputGray );
	stair_cascade.detectMultiScale(  inputGray , rectangles, 1.1, 1, 0|CV_HAAR_SCALE_IMAGE, cv::Size(90,150 ) );
	// draw rectangles on the output image
	if (pub_detect.getNumSubscribers() != 0)
	{
	cv_bridge::CvImage cv_det(cv_input_ptr->header,"bgr8",cv_input_ptr->image.clone());
	ROS_INFO("trying to publish");
	for( size_t i = 0; i < rectangles.size(); i++ )
	{
		cv::rectangle(cv_det.image, rectangles[i], cv::Scalar( 255, 0, 255 ), 4, 8,0);
	}
	pub_detect.publish(cv_det.toImageMsg());
	}

}
