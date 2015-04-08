#include "pointcloudregistration/vision.h"
#include <opencv2/opencv.hpp>

Vision::Vision():nh("~"),it(nh)
{
	image_lsd = it.advertise("lsd",1);
	//image_detect = it.advertise("detect",1);
	ls = cv::createLineSegmentDetector(cv::LSD_REFINE_STD);
	//stair_cascade_name = "stair_cascade.xml";
	//if( !stair_cascade.load( stair_cascade_name ) ){ printf("--(!)Error loading\n");  };


	wantExit=false;
	data_ready=false;
	//initiate lsd image
	cv_lsd_ptr = boost::make_shared<cv_bridge::CvImage>();
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
		//detect();
	}
}
void Vision::getLines()
{

	//assigning complete header at once should work to
	cv_lsd_ptr->header.stamp =cv_input_ptr->header.stamp;
	cv_lsd_ptr->header.seq =cv_input_ptr->header.seq;
	//convert to grayscale
	cv::imwrite("/home/rosuser/Downloads/input.png",cv_input_ptr->image);
	cv_lsd_ptr->image=cv_input_ptr->image.clone();
	cv::cvtColor(cv_input_ptr->image,InputGray,cv::COLOR_BGR2GRAY);
	cv::imwrite("/home/rosuser/Downloads/inputclone.png",cv_lsd_ptr->image);
	//detect using lsd
	ls->detect(InputGray, lines_std);
	ROS_INFO_STREAM("detected "<< lines_std.size() << " lines");

	ls->drawSegments(cv_lsd_ptr->image, lines_std);
	/*for(size_t i=0; i <  lines_std.size() ; i++)
	{
		cv::line(cv_lsd_ptr->image,cv::Point(lines_std[i][0],lines_std[i][1]),cv::Point(lines_std[i][2],lines_std[i][3]),cv::Scalar(0,0,255));
}*/
	cv::imwrite("/home/rosuser/Downloads/lines.png",cv_lsd_ptr->image);
	image_lsd.publish(cv_lsd_ptr->toImageMsg());
}
/*void Vision::detect()
{
	std::vector<cv::Rect> stairs;
	cv::equalizeHist( InputGray, InputGray );

  //-- Detect faces
	stair_cascade.detectMultiScale(  InputGray , stairs, 1.1, 2, 0|CV_HAAR_SCALE_IMAGE, cv::Size(40, 40) );

  for( size_t i = 0; i < stairs.size(); i++ )
  {
	cv::rectangle(cv_input_ptr->image, stairs[i], cv::Scalar( 255, 0, 255 ), 4, 8,0);
  }
	image_detect.publish(cv_input_ptr->toImageMsg());
}*/
void Vision::process(const sensor_msgs::ImageConstPtr& msg)
{
	//accept the new data
	{
		boost::mutex::scoped_lock lock(imageMutex);
	//copy the msg to a cv::Mat instance
	try
	{
		cv_input_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
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
	boost::mutex::scoped_lock lock(imageMutex, boost::try_to_lock);
	if(lock)
	{
		return true;
	}else{
		return false;
	}
}
