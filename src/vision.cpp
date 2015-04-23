#include "pointcloudregistration/vision.h"
#include "pointcloudregistration/settings.h"
#include <opencv2/opencv.hpp>
#include "pointcloudregistration/linereg.h"
#include <math.h>       /* tan */
#include "ros/package.h"


Vision::Vision(LineReg* stair): stairs(stair),nh("~"),it(nh),cv_lsd_ptr(new cv_bridge::CvImage),cv_det_ptr(new cv_bridge::CvImage)
{
	pub_lsd = it.advertise("lsd",1);
	pub_detect = it.advertise("detect",1);
	ls = cv::createLineSegmentDetector(cv::LSD_REFINE_ADV);
	stair_cascade_name = ros::package::getPath("pointcloudregistration")+"/src/stair_cascade.xml";
	if( !stair_cascade.load( stair_cascade_name ) )
		{ROS_WARN_STREAM("Error loading cascade: " << stair_cascade_name);  };
	wantExit=false;
	data_ready=false;
	//initiate lsd image
	cv_lsd_ptr->encoding="bgr8";
	cv_lsd_ptr->header.frame_id="line_segment";
	cv_det_ptr->encoding="bgr8";
	//start worker
	worker = boost::thread(boost::bind(&Vision::threadLoop,this));
}
Vision::~Vision()
{
	std::cout<<"called vision destructor"<< std::endl;
	//destroy thread
	{
		boost::mutex::scoped_lock lock(imageMutex);
		wantExit=true;
		newData.notify_all();
	}
	std::cout<<"waiting for thread to close"<< std::endl;
	worker.join();
	std::cout<<"visor destroyed"<< std::endl;
	//dtor
}
void Vision::threadLoop()
{
///
/// \brief This method is started in a different thread and waits for data. Upon reception of new data the methods
///Vision::getLines() and Vision::detect() are called.
///
	ROS_INFO_STREAM("detector constructed");
	while(true)
	{
		boost::mutex::scoped_lock lock(imageMutex);
		while(!data_ready)//check if new message passed
		{
			newData.wait(lock);
		}
		data_ready=false;
		if(wantExit)
			return;
		getLines();
		detect();
		stairs->process(rectangles,lines, quality,cv_input_ptr->image.clone());
	}
}
void Vision::getLines()
{
///
/// \brief Detects lines using the line segment detector implemented in OpenCV.
///
	cv_lsd_ptr->header =cv_input_ptr->header;
	//convert to grayscale
	cv_lsd_ptr->image=cv_input_ptr->image.clone();
	cv::cvtColor(cv_input_ptr->image,InputGray,cv::COLOR_BGR2GRAY);
	std::vector<cv::Vec4f> bad_lines;
	std::vector<float> quality_std;
	//detect using lsd
	ls->detect(InputGray, lines,cv::noArray(),cv::noArray(),quality_std);
	float maxtan = std::tan(static_cast<float>(maxAngle)/180.0f * CV_PI);
	auto iter(remove_if(lines.begin(),lines.end(),[&maxtan](cv::Vec4f& line) {return abs((line[3] - line[1])/(line[2]-line[0]))>maxtan;} ));
	bad_lines.insert(bad_lines.begin(),iter,lines.end());
	lines.erase(iter,lines.end());

	if (pub_lsd.getNumSubscribers() != 0)
	{
		float y =(cv_input_ptr->image.cols)/2.0f*tan(pose->roll/180.0*CV_PI);
		float y1= static_cast<float> (cv_lsd_ptr->image.rows) / 2.0f + y;
		float y2= static_cast<float> (cv_lsd_ptr->image.rows) / 2.0f - y;
		ls->drawSegments(cv_lsd_ptr->image,bad_lines);
		cv::line(cv_lsd_ptr->image,cv::Point2f(0.0f,y1),cv::Point2f(static_cast<float>(cv_lsd_ptr->image.cols),y2),cv::Scalar(255,0,0));
		cv::Scalar color(0,255,0);
		for(size_t i=0 ; i < lines.size();i++)
		{
			cv::line(cv_lsd_ptr->image,cv::Point2f(lines[i][0],lines[i][1]),cv::Point2f(lines[i][2],lines[i][3]),color);
		}
		pub_lsd.publish(cv_lsd_ptr->toImageMsg());
	}
}

void Vision::detect()
{
///
/// \brief Detect stairs using a cascade classifier.
///
//cv::Mat rot =  cv::getRotationMatrix2D(Point2f(static_cast<float>(cv_input_ptr->image.rows)/2.0f, static_cast<float>(cv_input_ptr->image.cols)/2.0f),pose.roll, 1.0);
	cv_det_ptr->header =cv_input_ptr->header;
	cv_det_ptr->image=cv_input_ptr->image.clone();
	cv::equalizeHist( InputGray, InputGray );
	stair_cascade.detectMultiScale(  InputGray , rectangles, 1.1, 1, 0|CV_HAAR_SCALE_IMAGE, cv::Size(90,150 ) );
	// draw rectangles on the output image
	if (pub_detect.getNumSubscribers() != 0)
	{
	ROS_INFO("trying to publish");
	for( size_t i = 0; i < rectangles.size(); i++ )
	{
		cv::rectangle(cv_det_ptr->image, rectangles[i], cv::Scalar( 255, 0, 255 ), 4, 8,0);
	}
	pub_detect.publish(cv_det_ptr->toImageMsg());
	}

}
void Vision::process(const sensor_msgs::ImageConstPtr& msg, const tum_ardrone::filter_stateConstPtr& poseMsg)
{
///
/// \brief  Accepts the data of an image and copies the neccesary data to the adequate private variables.
/// @param[in] msg an image
/// 
	{
		boost::mutex::scoped_lock lock(imageMutex);
	//copy the msg to a cv::Mat instance
	try
	{
		cv_input_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
		pose=poseMsg;
		data_ready=true;
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}
	}
	ROS_INFO("process image");
	//notify thread
	newData.notify_one();
}
bool Vision::ready()
{
///
/// \brief Checks if the imageMutex is still locked and thus the worker thread still occupied with the previous task.
///
	boost::mutex::scoped_lock lock(imageMutex, boost::try_to_lock);
	if(lock)
	{
		return true;
	}else{
		return false;
	}
}
