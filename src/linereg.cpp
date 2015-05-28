#include "pointcloudregistration/linereg.h"
#include "pointcloudregistration/settings.h"
#include "tf_conversions/tf_eigen.h"
#include <pcl/filters/extract_indices.h>
#include <sensor_msgs/image_encodings.h>
#include <Eigen/Geometry>
#include <ros/time.h>
#include <pcl/common/transforms.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_line.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include "pointcloudregistration/pcl_registration.h"
#include "tbb/parallel_for_each.h"
#include <cmath>        // std::abs
#include <iterator>     // std::back_inserter
#include <sstream>      // std::stringstream, std::stringbuf
#include <time.h>       //filename with time



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
double rico;
double constant;
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
	float sqew;
	int nmbrOfLines;
	float lineratio;
	float planeratio;
	std::vector<int> lineInliers;
	std::vector<int> planeInliers;
	cv::Point2d VP;
};
LineReg::LineReg(): nh("~"),it(nh),frame_id(0),can_id(0)
{
	pub_det = it.advertise("lines_curv",1);
	pub_can = it.advertise("candidates",1);
	point_pub = nh.advertise<geometry_msgs::PointStamped>("target",3);
	if(perfmon){
	  time_t t = time(0);   // get time now
	  struct tm * now = localtime( & t );
	  std::stringstream ss;

	  ss << (now->tm_year + 1900) << '-'
	     << (now->tm_mon + 1) << '-'
	     <<  now->tm_mday << '_'
	     << now->tm_hour << ':'
	     << now->tm_min << ':'
	     << now->tm_sec;
	//dir_path <<
	dir =boost::filesystem::path(ss.str());
	boost::filesystem::create_directories(dir);
	ROS_INFO_STREAM("creating directory: " << boost::filesystem::canonical(dir).string());

	ss<<"/results.csv";
		perfres.open (ss.str(),std::ofstream::out );
		perfres << "\"frame id\", \"can id\", \"MSE\", \"number of lines\", \"angle\", \"sqew\", \"TP\"" << std::endl;
	}
//ctor
}

LineReg::~LineReg()
{
		perfres.close();
//dtor
}
bool LineReg::operator()(cv::UMat  depthImg, cv::UMat  H, cv::UMat CI, std::vector<cv::Rect> rectangles, std::vector<cv::Vec4f>  lines, cv_bridge::CvImagePtr cv_input_ptr, const lsd_slam_viewer::keyframeMsgConstPtr frameMsg, pcl::PointCloud<pcl::PointXYZ>::Ptr& planeCloud, Eigen::Affine3f& pose)
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
/// \return True if there are candidates detected, false if not
	frame_id++;
	std::vector<Candidate> candidates;
	std::vector<DepthLine> depthLines;
	candidates.reserve(rectangles.size());
	for_each(rectangles.begin(),rectangles.end(),[&candidates](cv::Rect & rectangle){candidates.push_back(Candidate(rectangle));});
	depthLines.reserve(lines.size());
	for_each(lines.begin(),lines.end(),[&depthLines](cv::Vec4f & line ){depthLines.push_back(DepthLine(line));});
	cv::UMat curv_weight;
	cv::add(CI.mul(0.95f),0.05f,curv_weight);
/// Get inverse camera parameters
	float fxi=1.0f/frameMsg->fx;
	float fyi=1.0f/frameMsg->fy;
	float cxi=-frameMsg->cx * fxi;
	float cyi=-frameMsg->cy *fyi;
	cv::Mat linesImg, canImg;
/// initialize output images
	if(pub_det.getNumSubscribers() != 0)
			linesImg=cv_input_ptr->image.clone();

	if(pub_can.getNumSubscribers() != 0)
		canImg=cv_input_ptr->image.clone();

/// construct transformation matrix
	tf::StampedTransform transform;
	try{
		listener.lookupTransform("/map", "/tum_base_frontcam", cv_input_ptr->header.stamp, transform);
	}
	catch (tf::TransformException ex){
		ROS_ERROR("%s",ex.what());
		ros::Duration(1.0).sleep();
		return false;
	}
	Eigen::Affine3d transformeig;
	tf::transformTFToEigen(transform,transformeig);
	Eigen::Matrix4f trans =transformeig.matrix().cast<float>();
	Sophus::Sim3f camToWorld;
	memcpy(camToWorld.data(), frameMsg->camToWorld.data(), 7*sizeof(float));
	pose = camToWorld.matrix();
	//pose = pose.inverse();
	tbb::parallel_for_each(candidates.begin(),candidates.end(),[&](Candidate & can){
		getParallelLines(can, depthLines);
		if(can.nmbrOfLines==0)
			return;
		get3DLines(can, depthImg.getMat(cv::ACCESS_READ), curv_weight.getMat(cv::ACCESS_READ), H.getMat(cv::ACCESS_READ), fxi, fyi, cxi, cyi);
		if(can.lineInliers.empty())
			return;
		getPlane(can, linesImg, canImg, trans);
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
	candidates.erase(remove_if(candidates.begin(),candidates.end(),[](Candidate& can){return can.nmbrOfLines==0;}),candidates.end());

	if(candidates.size()>0){
		geometry_msgs::PointStamped target;
		target.header.stamp=ros::Time::now();
		target.header.frame_id="tum_base_frontcam";
		cv::Mat perf=cv_input_ptr->image.clone();
		for(auto can:candidates)
		{
			if(can.planeInliers.size()<10)
				continue;
			pcl::PointCloud<pcl::PointXYZ> temp(*can.cloud,can.planeInliers);
			*planeCloud+=temp;
			pcl::PointXYZ targetpt=std::accumulate(temp.begin(), temp.end(), pcl::PointXYZ(1000.0,1000.0,1000.0), [](pcl::PointXYZ result, pcl::PointXYZ& point){
				if(point.y< result.y){
					result.x=point.x;
					result.y=point.y;
					result.z=point.z;
				}
				return result;
			});
			if(targetpt.y==1000.0)
				continue;
			target.point.x=targetpt.x;
			target.point.y=targetpt.y;
			target.point.z=targetpt.z;
			point_pub.publish(target);
			if(perfmon){
				can_id++;
				perfres<<frame_id<<", "<< can_id << ", " << can.bestMSE << ", " << can.nmbrOfLines << ", " << can.angle << ", " << can.sqew << ", " << 1 << std::endl;
				std::stringstream fname;
				fname << boost::filesystem::canonical(dir).string() <<"/"<< frame_id<< ".png";
				std::stringstream canstr;
				canstr<<can_id;
				cv::Point origin(can.rectangle.x,can.rectangle.y);
				cv::putText(perf, canstr.str(), origin, cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0,0,255));
				cv::Point2d targetproj;
				targetproj.x=(frameMsg->fx*targetpt.x/targetpt.z+frameMsg->cx);
		                targetproj.y=(frameMsg->fy*targetpt.y/targetpt.z+frameMsg->cy);
				cv::rectangle(perf, can.rectangle, cv::Scalar(0,255,0));
				cv::circle(perf, targetproj, 2, cv::Scalar(0,255,0));
				cv::imwrite(fname.str(),perf);
			}
		}
		return true;
	}
	return false;
}
void LineReg::getParallelLines(Candidate & can, std::vector<DepthLine> & depthLines)
{
/// \brief Get the lines that are parallel with each other

	can.lines.reserve(depthLines.size());
/// get lines that intersect with rectangle
	for(auto lineit = depthLines.begin();lineit != depthLines.end(); lineit++)
	{
		if(SegmentIntersectRectangle(can.rectangle, lineit->line))
			can.lines.push_back(*lineit);
	}
/// calculate parameters of each line
	for_each(can.lines.begin(),can.lines.end(),[](DepthLine & line){
		line.rico=(line.line[3]-line.line[1])/(line.line[2]-line.line[0]);
		line.constant=line.line[1]-line.rico*line.line[0];
	});

	auto set=can.lines;
	ROS_INFO_STREAM("number of lines that intersect: "<< set.size());
	if(set.size()<minLines)
		return;
	random_shuffle(set.begin(), set.end());

	int iter(0),maxit(set.size());
	std::vector<DepthLine> alsoinliers;
	alsoinliers.reserve(set.size());
	unsigned int maxinliers(0);
	while(iter<maxit)
	{
		iter++;
		alsoinliers.clear();
		/// get maybeinliers from set
		DepthLine line1=set[0];
		DepthLine line2=set[1];
		/// shift set every iteration
		std::rotate(set.begin(), set.begin()+1, set.end());	
		if(std::abs(line1.rico-line2.rico)<0.00001){
			ROS_INFO("lines are exactly parallel");
			continue;
		}
		cv::Point2d mod;
		/// get model of maybeinliers
		mod.x= (line1.constant-line2.constant)/(line2.rico-line1.rico);
		mod.y= mod.x*line1.rico+line1.constant;
		/// add maybeinliers to alsoinliers
		alsoinliers.push_back(line1);
		alsoinliers.push_back(line2);
		/// add elements from set to alsoinliers if intersection with one of the maybeinliers is close enough to the model
		copy_if(set.begin()+1, set.end()-1, back_inserter(alsoinliers),[&mod,&line1,&line2](DepthLine & line ){
			double x= (line1.constant-line.constant)/(line.rico-line1.rico);
			cv::Point2d inter1(x, line1.rico*x+line1.constant);
			x= (line2.constant-line.constant)/(line.rico-line2.rico);
			cv::Point2d inter2(x, line2.rico*x+line2.constant);
			double dist1 = pow(mod.x-inter1.x,2)+pow(mod.y-inter1.y,2);
			double dist2 = pow(mod.x-inter2.x,2)+pow(mod.y-inter2.y,2);
			return std::min(dist1,dist2)<TH;
		});
		/// check if there are enough inliers
		if(alsoinliers.size()<minLines)
			return;
		/// get all intersections
		std::vector<cv::Point2d> intersections;
		intersections.reserve((std::pow(alsoinliers.size(),2)-alsoinliers.size())/2.0);
		for(auto it = alsoinliers.begin(); it+1 != alsoinliers.end(); it++)
		{
			std::for_each(it+1,alsoinliers.end(),[&intersections,&it](DepthLine& line){
				if(std::abs(it->rico-line.rico)<0.00001){
					ROS_INFO("lines are exactly parallel");
					return;
				}
				double x= (it->constant-line.constant)/(line.rico-it->rico);
				intersections.emplace_back(x, it->rico*x+it->constant);
			});
		}
		/// get the mean intersection
		ROS_INFO_STREAM("predicted n/o intersects: "<< (std::pow(alsoinliers.size(),2)-alsoinliers.size())/2.0<< "n/o intersects: "<< intersections.size());
		cv::Point2d sum=accumulate(intersections.begin(),intersections.end(),cv::Point2d(0.0,0.0),[](cv::Point2d& result, cv::Point2d& point){return result + point;});
		cv::Point2d mean= cv::Point2d(sum.x/intersections.size(), sum.y/intersections.size());
		ROS_INFO_STREAM("intersectionmod: "<< mod.x << ", " << mod.y << "mean: " << mean.x << ", " << mean.y << " intersection "<< intersections.size()<< "alsoinliers: "<< alsoinliers.size());
		double SSE= accumulate(intersections.begin(),intersections.end(),0.0,[&mean](double result, cv::Point2d point){return result+pow(mean.x-point.x,2)+pow(mean.y-point.y,2);});
		double MSE=SSE/intersections.size();
		if(set.size() == alsoinliers.size()){
			if(set.size()>minLines)
			{
			ROS_INFO("all are inliers");
			can.nmbrOfLines=alsoinliers.size();
			can.bestMSE=MSE;
			can.VP=mean;
			}
			break;
		}
		if(alsoinliers.size()>minLines )
		{
			ROS_INFO_STREAM("SSE: "<< SSE << " size: "<< alsoinliers.size() << " model: " << mean.x << ", " << mean.y);
			if(alsoinliers.size()>maxinliers)
			{
				maxinliers=alsoinliers.size();
				ROS_INFO_STREAM("maxinliers "<< maxinliers);
				maxit=std::min(static_cast<int>(std::log(eps)/std::log(1.0f-static_cast<float>(maxinliers)/static_cast<float>(set.size()))+0.5f), static_cast<int>(set.size()));
			}
			if(MSE<can.bestMSE)
			{
				ROS_INFO_STREAM("update_MSE ("<< MSE << ") ,maxit "<< maxit);
				can.bestMSE=MSE;
				can.nmbrOfLines+=alsoinliers.size();
				can.lines=alsoinliers;
				can.VP=mean;
			}
		}

	}

}
void LineReg::get3DLines(Candidate& can,const cv::Mat& depthImg,const cv::Mat& curv_weight, const cv::Mat& meanCurvature, float & fxi, float & fyi, float & cxi, float & cyi)
{
/// \brief get 3D points from lines and depthImage.
/// @param[in] can candidate with initialised rectangle
/// @param[in] depthImg filtered depth image
/// @param[in] curv_weight weighted curvature
/// @param[in] meanCurvature mean curvature 
/// @param[in] fxi inverse focal length in x direction
/// @param[in] fyi inverse focal length in y direction
/// @param[in] cxi inverse first ordinate of the camera’s principal point
/// @param[in] cyi inverse second ordinate of the camera’s principal point

/// set line properties
ROS_INFO("getting 3D lines");
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
		if(it->points.size()<5)
		{
			continue;
		}
/// created RandomSampleConsensus object and compute the line model for each line
	pcl::SampleConsensusModelLine<pcl::PointXYZ>::Ptr model_l (new pcl::SampleConsensusModelLine<pcl::PointXYZ> (can.cloud,it->points));
	pcl::RandomSampleConsensus<pcl::PointXYZ> ransac (model_l);
	ransac.setDistanceThreshold (.1);
	ransac.computeModel();
	ransac.getInliers(it->inliers);
	can.lineInliers.insert(can.lineInliers.end(),it->inliers.begin(),it->inliers.end());
	}
}
void LineReg::getPlane(Candidate& can, cv::Mat& linesImg, cv::Mat& canImg, const Eigen::Matrix4f& trans)
{
/// \brief Do robust estimation from the plane that virtually lays on the staircase
/// @param[in] can candidate containing 3D-lines
/// @param[out] linesImg output image containing lines with curvature
/// @param[out] canImg output image containing the candidates with extra info
/// @param[in] trans rigid body transform between camera and ekf

	pcl::PointCloud<pcl::PointXYZ>::Ptr ekfcloud;
	pcl::transformPointCloud(*can.cloud,*ekfcloud,trans);
	pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr model_p (new pcl::SampleConsensusModelPlane<pcl::PointXYZ> (ekfcloud, can.lineInliers));
	Eigen::VectorXf coefficients;
	pcl::RandomSampleConsensus<pcl::PointXYZ> ransac (model_p);
	ransac.setDistanceThreshold (.02);
	ransac.computeModel();
	ransac.getInliers(can.planeInliers);
	ransac.getModelCoefficients(coefficients);
	can.angle = std::abs(std::acos(coefficients(2)));
	if(can.angle > CV_PI/2.0f)
		can.angle-=CV_PI/2.0f;
	ROS_INFO_STREAM("angle (deg) " <<  can.angle*180.0f/CV_PI);
	if(can.angle<minStairAngle || can.angle>maxStairAngle){
		can.nmbrOfLines=0;
		return;
	}
	Eigen::Vector4f Ptmin, Ptmax;
	pcl::getMinMax3D(*ekfcloud, can.planeInliers, Ptmin, Ptmax);
	ROS_INFO_STREAM("boundingbox: "<< Ptmin << ", " << Ptmax);
	coefficients(3)=1;
	Eigen::VectorXf coefcam= trans.inverse()*coefficients;
	can.sqew=coefcam(1);
	if(!canImg.empty())
	{
		cv::rectangle(canImg, can.rectangle, cv::Scalar(0,255,0));
		std::stringstream data;
		data << "angle: " << can.angle*180.0/CV_PI << "\n"
			 << "nmbr of lines: " << can.nmbrOfLines << "\n"
			<< "ratio: " << static_cast<float>(can.planeInliers.size())/static_cast<float>(can.lineInliers.size()) << "\n"
			<< "MSE: "<< can.bestMSE << "\n"
			<< "sqew: "<< can.sqew << "\n";
		int baseline=0;
		cv::Point origin(can.rectangle.x,can.rectangle.y);
		std::string s = data.str();
		std::string delimiter = "\n";
		size_t pos = 0;
		std::string token;
		while ((pos = s.find(delimiter)) != std::string::npos) {
			token = s.substr(0, pos);
			cv::Size textSize = cv::getTextSize(token, cv::FONT_HERSHEY_SIMPLEX, 0.4, 1, &baseline);
			origin+=cv::Point(0,baseline+1+ textSize.height);
			cv::putText(canImg, token, origin, cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0,0,255));
			s.erase(0, pos + delimiter.length());
		}
	}

	if(!linesImg.empty())
	{
		for(size_t i=0 ; i < can.lines.size();i++)
		{
			cv::Vec4f line =can.lines[i].line;
			cv::Scalar color(0, 255*can.lines[i].curvature, 255*(1.0f-can.lines[i].curvature));
			cv::line(linesImg, cv::Point2f(line[0],line[1]), cv::Point2f(line[2],line[3]),color);
			cv::line(linesImg, cv::Point2f(line[0],line[1]),can.VP,cv::Scalar(255,0,0));
		}
	}
}
bool LineReg::SegmentIntersectRectangle(cv::Rect& rectangle, cv::Vec4f& line)
{
///
/// \brief Check if a line intersects with a rectangle.
/// @param[in] rectangle a rectangle defined by origin and size
/// @param[in] line line defined by begin and endpoint
double a_p1x = static_cast<double>(line[0]);
double a_p1y = static_cast<double>(line[1]);
double a_p2x = static_cast<double>(line[2]);
double a_p2y = static_cast<double>(line[3]);
double a_rectangleMinX = static_cast<double>(rectangle.x);
double a_rectangleMinY = static_cast<double>(rectangle.y);
double a_rectangleMaxX = static_cast<double>(rectangle.x+rectangle.width);
double a_rectangleMaxY = static_cast<double>(rectangle.y+rectangle.height);

/// Find min and max X for the segment

    double minX = a_p1x;
    double maxX = a_p2x;

    if(a_p1x > a_p2x){
      minX = a_p2x;
      maxX = a_p1x;
    }

/// Find the intersection of the segment's and rectangle's x-projections
    if(maxX > a_rectangleMaxX){
      maxX = a_rectangleMaxX;
    }

    if(minX < a_rectangleMinX){
      minX = a_rectangleMinX;
    }
/// If their projections do not intersect return false
    if(minX > maxX){
      return false;
    }

/// Find corresponding min and max Y for min and max X we found before

    double minY = a_p1y;
    double maxY = a_p2y;

    double dx = a_p2x - a_p1x;

    if(std::abs(dx) > 0.0000001){
      double a = (a_p2y - a_p1y) / dx;
      double b = a_p1y - a * a_p1x;
      minY = a * minX + b;
      maxY = a * maxX + b;
    }

    if(minY > maxY){
      double tmp = maxY;
      maxY = minY;
      minY = tmp;
    }

/// Find the intersection of the segment's and rectangle's y-projections

    if(maxY > a_rectangleMaxY){
      maxY = a_rectangleMaxY;
    }

    if(minY < a_rectangleMinY){
      minY = a_rectangleMinY;
    }
/// If Y-projections do not intersect return false
    if(minY > maxY){
      return false;
    }
    return true;
  }
