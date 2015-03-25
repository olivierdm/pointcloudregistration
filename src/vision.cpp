#include "pointcloudregistration/vision.h"


Vision::Vission():nh("~"),it(nh)
{
	image_edge_ = it.advertise("edge", 1);
	image_hough_ = it.advertise("hough", 1);
	image_lsd_ = it.advertise("lsd",1);
	ls = cv::createLineSegmentDetector(cv::LSD_REFINE_STD);
   //initiate edge image
    cv_edge_ptr = boost::make_shared<cv_bridge::CvImage>();
    cv_edge_ptr->encoding="mono8";
    cv_edge_ptr->header.frame_id="edge_detector";

    //initiate hough image
    cv_hough_ptr = boost::make_shared<cv_bridge::CvImage>();
    cv_hough_ptr->encoding="bgr8";
    cv_hough_ptr->header.frame_id="hough_lines";

    //initiate lsd image
    cv_lsd_ptr = boost::make_shared<cv_bridge::CvImage>();
    cv_lsd_ptr->encoding="bgr8";
    cv_lsd_ptr->header.frame_id="line_segment";
    ROS_INFO_STREAM("detector constructed");
}

Vision::addImgMsg(const sensor_msgs::ImageConstPtr& msg)
{
	imgPtr.lock();
	imgMsg=msg;
	imgPtr.unlock();
}
Vision::process()
{
	imgPtr.lock();
 try
     {
       cv_input_ptr = cv_bridge::toCvCopy(imgMsg, sensor_msgs::image_encodings::BGR8);
     }
     catch (cv_bridge::Exception& e)
     {
       ROS_ERROR("cv_bridge exception: %s", e.what());
       return;
     }
	imgPtr.unlock();
   cv_edge_ptr->header.stamp =cv_input_ptr->header.stamp;
     cv_edge_ptr->header.seq =cv_input_ptr->header.seq;
     cv_hough_ptr->header.stamp =cv_input_ptr->header.stamp;
     cv_hough_ptr->header.seq =cv_input_ptr->header.seq;
     cv_lsd_ptr->header.stamp =cv_input_ptr->header.stamp;
     cv_lsd_ptr->header.seq =cv_input_ptr->header.seq;
     //convert to grayscale
     cv::cvtColor(cv_input_ptr->image,InputGray,cv::COLOR_BGR2GRAY);
 //smoothing
    cv::GaussianBlur(InputGray,BlurredImg,cv::Size(3,3),0,0);
    //edge extraction
    double otsu_thresh_val = cv::threshold(BlurredImg, ThreshImg, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU );
    double high_thresh_val  = otsu_thresh_val,
       lower_thresh_val = otsu_thresh_val * 0.5;
    cv::Canny(BlurredImg,cv_edge_ptr->image,lower_thresh_val,high_thresh_val);
//publish edges
    image_edge_.publish(cv_edge_ptr->toImageMsg());

    std::vector<cv::Vec2f> lines;
    double minTheta=7.0*CV_PI/16.0,maxTheta=9.0*CV_PI/16.0 ;
    cv::HoughLines(cv_edge_ptr->image, lines,2, CV_PI/120, 200, 0, 0, minTheta,maxTheta);
    cv_hough_ptr->image=cv_input_ptr->image.clone();
    cv::Mat hlines=cv::Mat::zeros(cv_input_ptr->image.rows,cv_input_ptr->image.cols,CV_8UC1);
    for( size_t i = 0; i < lines.size(); i++ )
        {
            float rho = lines[i][0], theta = lines[i][1]+minTheta;
            cv::Point pt1, pt2;
            double a = cos(theta), b = sin(theta);
            double x0 = a*rho, y0 = b*rho;
            pt1.x = cvRound(x0 + 1000*(-b));
            pt1.y = cvRound(y0 + 1000*(a));
            pt2.x = cvRound(x0 - 1000*(-b));
            pt2.y = cvRound(y0 - 1000*(a));
            cv::line( cv_hough_ptr->image, pt1, pt2, cv::Scalar(0,0,255), 1,cv::LINE_AA);
            cv::line( hlines, pt1, pt2, cv::Scalar(255), 2,cv::LINE_8);
        }
   //  cv::bitwise_and(Edges,hlines,edgesH);
    image_hough_.publish(cv_hough_ptr->toImageMsg());

    //detect using lsd
    ls->detect(InputGray, lines_std);
    cv_lsd_ptr->image=cv_input_ptr->image.clone();
    ls->drawSegments(cv_lsd_ptr->image, lines_std);
    image_lsd_.publish(cv_lsd_ptr->toImageMsg());
   }
}
