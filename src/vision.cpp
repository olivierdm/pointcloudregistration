#include "pointcloudregistration/vision.h"


Vision::Vision():nh("~"),it(nh)
{
	image_lsd = it.advertise("lsd",1);
	ls = cv::createLineSegmentDetector(cv::LSD_REFINE_STD);

	//initiate lsd image
	cv_lsd_ptr = boost::make_shared<cv_bridge::CvImage>();
	cv_lsd_ptr->encoding="bgr8";
	cv_lsd_ptr->header.frame_id="line_segment";
	//start worker
	worker = boost::thread(boost::bind(&Vision::threadLoop,this));
}
Vision::~Vision()
{
	//destroy thread
	{
		boost::mutex::scoped_lock lock(imageMutex);
		wantExit=true;
		newData.notify_one();
	}
	worker.join();
	//dtor
}
void Vision::threadLoop()
{
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
	}
}
void Vision::getLines()
{

	//assigning complete header at once should work to
	cv_lsd_ptr->header.stamp =cv_input_ptr->header.stamp;
	cv_lsd_ptr->header.seq =cv_input_ptr->header.seq;
	//convert to grayscale
	cv::cvtColor(cv_input_ptr->image,InputGray,cv::COLOR_BGR2GRAY);
	//detect using lsd
	ls->detect(InputGray, lines_std);
	cv_lsd_ptr->image=cv_input_ptr->image.clone();
	ls->drawSegments(cv_lsd_ptr->image, lines_std);
	image_lsd.publish(cv_lsd_ptr->toImageMsg());
}
void Vision::process(const sensor_msgs::ImageConstPtr& msg)
{
	//accept the new data
	{
		boost::mutex::scoped_lock lock(imageMutex);
	//copy the msg to a cv::Mat instance
	try
	{
		cv_input_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}
	}
	//notify thread
	newData.notify_one();
}
bool Vision::ready()
{
	boost::mutex::scoped_lock lock(imageMutex, boost::try_to_lock);
	if(lock)
	{
		return true;
	}else{
		return false;
	}
}
