#include "pointcloudregistration/linereg.h"
#include "pointcloudregistration/settings.h"
#include <pcl/filters/extract_indices.h>
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <Eigen/Geometry>
#include <pcl/common/transforms.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_line.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include "pointcloudregistration/pcl_registration.h"
#include "tbb/parallel_for_each.h"
#include <cmath>        // std::abs
#include <iterator>     // std::back_inserter
#include <sstream>      // std::stringstream, std::stringbuf


struct DepthLine
{
DepthLine(cv::Vec4f & lin):line(lin),curvature(0.0),rico(6.0f)
{
	//ctor
}
cv::Vec4f line;
std::vector<int> points;
std::vector<int> inliers;
double curvature;
bool convex;
float rico;
};
struct Candidate
{
	Candidate(cv::Rect & rect): rectangle(rect),cloud(new pcl::PointCloud<pcl::PointXYZ>),bestMSE(FLT_MAX),nmbrOfLines(0)
	{
		//ctor
	}
	cv::Rect rectangle;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
	std::vector<DepthLine> lines;
	float bestMSE;
	float angle;
	int nmbrOfLines;
	float lineratio;
	float planeratio;
	std::vector<int> lineInliers;
	std::vector<int> planeInliers;
};
LineReg::LineReg(): nh("~"),it(nh)
{
	pub_det = it.advertise("lines_curv",1);
	pub_can = it.advertise("candidates",1);
//ctor
}

LineReg::~LineReg()
{
//dtor
}
bool LineReg::operator()(cv::UMat  depthImg, cv::UMat &  H, cv::UMat & CI, std::vector<cv::Rect> & rectangles, std::vector<cv::Vec4f> &  lines, cv_bridge::CvImagePtr & cv_input_ptr, const lsd_slam_viewer::keyframeMsgConstPtr &frameMsg, const tum_ardrone::filter_stateConstPtr &poseMsg)
{
///
/// \brief  Accepts the data and fusions it to expell candidates
/// @param[in] depthImg a filtered version of the rendered depth image
/// @param[in] H matrix with the mean curvature
/// @param[in] CI matrix containing the curvature in each point
/// @param[in] rectangles vector containing all the candidates
/// @param[in] lines set of lines that are detected on the visual image
/// @param[in] cv_input_ptr the colored input image
/// @param[in] frameMsg lsdslame liveframe
/// @param[in] poseMsg state information coming from TUM_ardrone

	std::vector<Candidate> candidates;
	std::vector<DepthLine> depthLines;
	candidates.reserve(rectangles.size());
	for_each(rectangles.begin(),rectangles.end(),[&candidates](cv::Rect & rectangle){candidates.push_back(Candidate(rectangle));});
	depthLines.reserve(lines.size());
	for_each(lines.begin(),lines.end(),[&depthLines](cv::Vec4f & line ){depthLines.push_back(DepthLine(line));});
	cv::UMat curv_weight;
	cv::add(CI.mul(0.95f),0.05f,curv_weight);
	float fxi=1.0f/frameMsg->fx;
	float fyi=1.0f/frameMsg->fy;
	float cxi=-frameMsg->cx * fxi;
	float cyi=-frameMsg->cy *fyi;
	cv::Mat linesImg, canImg;
	//copy the cv_bridge instance to a cv::Mat if there are subscribers
	if(pub_det.getNumSubscribers() != 0)
			linesImg=cv_input_ptr->image.clone();

	if(pub_can.getNumSubscribers() != 0)
		canImg=cv_input_ptr->image.clone();

	tbb::parallel_for_each(candidates.begin(),candidates.end(),[&](Candidate & can){
		getParallelLines(can, depthLines);
		if(can.nmbrOfLines==0)
			return;
		get3DLines(can, depthImg.getMat(cv::ACCESS_READ), curv_weight.getMat(cv::ACCESS_READ), H.getMat(cv::ACCESS_READ), fxi, fyi, cxi, cyi);
		getPlane(can, linesImg, canImg, poseMsg);
		ROS_INFO("process candidate");
	});
	if(!linesImg.empty()){
		cv_bridge::CvImage cv_line(cv_input_ptr->header,"bgr8", linesImg);
		pub_det.publish(cv_line.toImageMsg());
	}
	if(!canImg.empty()){
		cv_bridge::CvImage cv_can(cv_input_ptr->header,"bgr8", canImg);
		pub_can.publish(cv_can.toImageMsg());
	}
	return false;
}
void LineReg::getParallelLines(Candidate & can, std::vector<DepthLine> & depthLines)
{

		//get lines that intersect with rectangle
		can.lines.reserve(depthLines.size());
		//copy_if(depthLines.begin(),depthLines.end(),back_inserter(can.lines),[&can,this](DepthLine& line){ROS_INFO_STREAM("test" << can.lines.size()); return SegmentIntersectRectangle(can.rectangle, line.line);});
		for(auto lineit = depthLines.begin();lineit != depthLines.end(); lineit++)
		{
			if(SegmentIntersectRectangle(can.rectangle, lineit->line))
				can.lines.push_back(*lineit);
		}
		//can.lines.resize(std::distance(can.lines.begin(),it));
		//calculate rico
		for_each(can.lines.begin(),can.lines.end(),[](DepthLine & line){line.rico=std::abs((line.line[3]-line.line[1])/(line.line[2]-line.line[0]));});
		int iter(0),maxit(10);
		auto maybeinliers=can.lines;
		random_shuffle(maybeinliers.begin(),maybeinliers.end());
		auto randit=maybeinliers.begin();
		float TH=0.2f;
		float eps=0.001f;
		unsigned int maxinliers(0);
		while(iter<maxit&&randit!=maybeinliers.end())
		{
			std::vector<DepthLine> alsoinliers;
			float mayberico = randit->rico;
			randit++;
			copy_if(maybeinliers.begin(),maybeinliers.end(),back_inserter(alsoinliers),[&mayberico,&TH](DepthLine & line ){return std::abs(line.rico-mayberico)<TH;});

			float sum = accumulate(alsoinliers.begin(),alsoinliers.end(),0.0f,[](float result, DepthLine& line){return result+line.rico;});
			float modelrico = sum/alsoinliers.size();
			float SSE= accumulate(alsoinliers.begin(),alsoinliers.end(),0.0f,[&modelrico](float result, DepthLine line){return result+pow(line.rico-modelrico,2);});
			float MSE=SSE/alsoinliers.size();
			if(maybeinliers.size() == alsoinliers.size()){
				if(maybeinliers.size()>minLines)
				{
				ROS_INFO("all are inliers");
				can.nmbrOfLines=alsoinliers.size();
				can.bestMSE=MSE;
				}
				break;
			}
			if(alsoinliers.size()>minLines )
			{
				ROS_INFO_STREAM("SSE: "<< SSE << " size: "<< alsoinliers.size() << " model: " << modelrico);
				if(alsoinliers.size()>maxinliers)
				{
					maxinliers=alsoinliers.size();
					ROS_INFO_STREAM("maxinliers "<< maxinliers);
					maxit=static_cast<int>(std::log(eps)/std::log(1.0f-static_cast<float>(maxinliers)/static_cast<float>(maybeinliers.size()))+0.5f);
				}
				if(MSE<can.bestMSE)
				{
					ROS_INFO_STREAM("update_MSE ("<< MSE << ") ,maxit "<< maxit);
					can.bestMSE=MSE;
					can.nmbrOfLines=alsoinliers.size();
					can.lines=alsoinliers;
				}
			}
		}

}
void LineReg::get3DLines(Candidate & can,cv::Mat depthImg, cv::Mat curv_weight,cv::Mat meanCurvature, float & fxi, float & fyi, float & cxi, float & cyi)
{
ROS_INFO("getting lines");
	//get points from lines and depthImage
		//set line properties
	for(auto it = can.lines.begin(); it != can.lines.end();it++)
	{
		cv::LineIterator line_it(depthImg,cv::Point(it->line[0],it->line[1]),cv::Point(it->line[2],it->line[3]));
		double H(0.0);		
		double nmbr=0.0;
		it->points.reserve(line_it.count);
		for(int j = 0; j < line_it.count; j++,++line_it)
		{
			cv::Point linePt=line_it.pos();
			float depth=depthImg.at<float>(linePt);
			if(std::abs(maxZ-depth)<0.5f)
				continue;
			nmbr++;
			it->curvature+=curv_weight.at<double>(linePt);
			H+=meanCurvature.at<double>(linePt);
			pcl::PointXYZ point;
	    	point.x=(linePt.x*fxi + cxi)*depth;
	    	point.y=(linePt.y*fyi + cyi)*depth;
	    	point.z=depth;
			can.cloud->push_back(point);
			it->points.push_back(static_cast<int>(can.cloud->size()));
		}
		it->convex=0.0<H;
		it->curvature/=nmbr;
		if(it->points.size()<10)
			continue;
	// created RandomSampleConsensus object and compute the appropriated model
	pcl::SampleConsensusModelLine<pcl::PointXYZ>::Ptr model_l (new pcl::SampleConsensusModelLine<pcl::PointXYZ> (can.cloud,it->points));
	pcl::RandomSampleConsensus<pcl::PointXYZ> ransac (model_l);
	ransac.setDistanceThreshold (.01);
	ransac.computeModel();
	ransac.getInliers(it->inliers);
	can.lineInliers.insert(can.lineInliers.end(),it->inliers.begin(),it->inliers.end());
	}
}
void LineReg::getPlane(Candidate & can, cv::Mat & linesImg, cv::Mat & canImg, const tum_ardrone::filter_stateConstPtr &poseMsg)
{
	if(can.lineInliers.empty())
		return;
	pcl::PointCloud<pcl::PointXYZ> ekfcloud;
	Eigen::Matrix4f trans =Eigen::Matrix4f::Identity();
	Eigen::AngleAxisd rotxAngle(-CV_PI/2.0, Eigen::Vector3d::UnitX());
	Eigen::AngleAxisd rotzAngle(-CV_PI/2.0, Eigen::Vector3d::UnitZ());
	Eigen::AngleAxisd rollAngle(poseMsg->roll, Eigen::Vector3d::UnitZ());
	Eigen::AngleAxisd yawAngle(poseMsg->yaw, Eigen::Vector3d::UnitY());
	Eigen::AngleAxisd pitchAngle(poseMsg->pitch, Eigen::Vector3d::UnitX());

	Eigen::Quaternion<double> q = rollAngle * yawAngle * pitchAngle*rotzAngle*rotxAngle;
	trans.block<3,3>(0,0) = q.matrix().cast<float>();
	trans(0,3)=poseMsg->x;
	trans(1,3)=poseMsg->y;
	trans(2,3)=poseMsg->z;

	pcl::transformPointCloud(*can.cloud,ekfcloud,trans);
	pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr model_p (new pcl::SampleConsensusModelPlane<pcl::PointXYZ> (can.cloud, can.lineInliers));
	Eigen::VectorXf coefficients;
	pcl::RandomSampleConsensus<pcl::PointXYZ> ransac (model_p);
	ransac.setDistanceThreshold (.01);
	ransac.computeModel();
	ransac.getInliers(can.planeInliers);
	ransac.getModelCoefficients(coefficients);
	can.angle = std::abs(std::acos(coefficients(2)));
	if(can.angle > CV_PI/2.0f)
		can.angle-=CV_PI/2.0f;
	ROS_INFO_STREAM("angle (deg) " <<  can.angle*180.0f/CV_PI);
	if(can.angle<minStairAngle || can.angle>maxStairAngle)
		return;
	if(!canImg.empty())
	{
		cv::rectangle(canImg, can.rectangle, cv::Scalar(0,255,0));
		std::stringstream first, second, third;
		first << "angle: " << can.angle*180.0/CV_PI;
		second << "nmbr of lines: " << can.nmbrOfLines;
		third << "MSE: "<< can.bestMSE;
		int baseline=0;
		cv::Point origin(can.rectangle.x,can.rectangle.y);
		cv::Size textSize = cv::getTextSize(first.str(), cv::FONT_HERSHEY_SIMPLEX, 0.4, 1, &baseline);
		origin+=cv::Point(0,baseline+1+ textSize.height);
		cv::putText(canImg, first.str(), origin, cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0,0,255));

		textSize = cv::getTextSize(second.str(), cv::FONT_HERSHEY_SIMPLEX, 0.4, 1, &baseline);
		origin+=cv::Point(0,baseline+1+textSize.height );
		cv::putText(canImg, second.str(), origin, cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0,0,255));

		textSize = cv::getTextSize(third.str(), cv::FONT_HERSHEY_SIMPLEX, 0.4, 1, &baseline);
		origin+=cv::Point(0,baseline+1+textSize.height);
		cv::putText(canImg, third.str(), origin, cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0,0,255));
	}

	if(!linesImg.empty())
	{
		for(size_t i=0 ; i < can.lines.size();i++)
		{
			cv::Vec4f line =can.lines[i].line;
			cv::Scalar color(0, 255*can.lines[i].curvature, 255*(1.0f-can.lines[i].curvature));
			cv::line(linesImg, cv::Point2f(line[0],line[1]), cv::Point2f(line[2],line[3]),color);
		}
	}
}
bool LineReg::SegmentIntersectRectangle(cv::Rect& rectangle, cv::Vec4f& line)
{
double a_p1x = static_cast<double>(line[0]);
double a_p1y = static_cast<double>(line[1]);
double a_p2x = static_cast<double>(line[2]);
double a_p2y = static_cast<double>(line[3]);
double a_rectangleMinX = static_cast<double>(rectangle.x);
double a_rectangleMinY = static_cast<double>(rectangle.y);
double a_rectangleMaxX = static_cast<double>(rectangle.x+rectangle.width);
double a_rectangleMaxY = static_cast<double>(rectangle.y+rectangle.height);

    // Find min and max X for the segment

    double minX = a_p1x;
    double maxX = a_p2x;

    if(a_p1x > a_p2x)
    {
      minX = a_p2x;
      maxX = a_p1x;
    }

    // Find the intersection of the segment's and rectangle's x-projections

    if(maxX > a_rectangleMaxX)
    {
      maxX = a_rectangleMaxX;
    }

    if(minX < a_rectangleMinX)
    {
      minX = a_rectangleMinX;
    }

    if(minX > maxX) // If their projections do not intersect return false
    {
      return false;
    }

    // Find corresponding min and max Y for min and max X we found before

    double minY = a_p1y;
    double maxY = a_p2y;

    double dx = a_p2x - a_p1x;

    if(std::abs(dx) > 0.0000001)
    {
      double a = (a_p2y - a_p1y) / dx;
      double b = a_p1y - a * a_p1x;
      minY = a * minX + b;
      maxY = a * maxX + b;
    }

    if(minY > maxY)
    {
      double tmp = maxY;
      maxY = minY;
      minY = tmp;
    }

    // Find the intersection of the segment's and rectangle's y-projections

    if(maxY > a_rectangleMaxY)
    {
      maxY = a_rectangleMaxY;
    }

    if(minY < a_rectangleMinY)
    {
      minY = a_rectangleMinY;
    }

    if(minY > maxY) // If Y-projections do not intersect return false
    {
      return false;
    }

    return true;
  }
