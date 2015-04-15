#include "pointcloudregistration/vision.h"
#include <opencv2/opencv.hpp>
#include <math.h>       /* tan */

Vision::Vision():nh("~"),it(nh),cv_lsd_ptr(new cv_bridge::CvImage)
{
	pub_lsd = it.advertise("lsd",1);
	pub_detect = it.advertise("detect",1);
	ls = cv::createLineSegmentDetector(cv::LSD_REFINE_STD);
	stair_cascade_name = "stair_cascade.xml";
	if( !stair_cascade.load( stair_cascade_name ) )
		{ROS_WARN_STREAM("Error loading cascade: " << stair_cascade_name);  };
	wantExit=false;
	data_ready=false;
	//initiate lsd image
	cv_lsd_ptr->encoding="bgr8";
	cv_lsd_ptr->header.frame_id="line_segment";
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
	//detect using lsd
	ls->detect(InputGray, lines_std);
	if (pub_lsd.getNumSubscribers() != 0)
	{
	ls->drawSegments(cv_lsd_ptr->image, lines_std);
	pub_lsd.publish(cv_lsd_ptr->toImageMsg());
	}

	
	float y =(cv_input_ptr->image.cols)/2.0f*tan(pose->roll);
	cv::line(cv_lsd_ptr->image,cv::Point2f(0.0f,y),cv::Point2f(0.0f,-y),cv::Scalar(255,0,0));

}
void Vision::detect()
{
///
/// \brief Detect stairs using a cascade classifier.
///
//cv::Mat rot =  cv::getRotationMatrix2D(Point2f(static_cast<float>(cv_input_ptr->image.rows)/2.0f, static_cast<float>(cv_input_ptr->image.cols)/2.0f),pose.roll, 1.0);
	std::vector<cv::Rect> stairs;
	cv::equalizeHist( InputGray, InputGray );
	stair_cascade.detectMultiScale(  InputGray , stairs, 1.1, 2, 0|CV_HAAR_SCALE_IMAGE, cv::Size(45, 75) );
	// draw rectangles on the output image
	if (pub_lsd.getNumSubscribers() != 0)
	{
	for( size_t i = 0; i < stairs.size(); i++ )
	{
		cv::rectangle(cv_input_ptr->image, stairs[i], cv::Scalar( 255, 0, 255 ), 4, 8,0);
	}
	pub_detect.publish(cv_input_ptr->toImageMsg());
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
