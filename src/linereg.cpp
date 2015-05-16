#include "pointcloudregistration/linereg.h"
#include "pointcloudregistration/settings.h"
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
	int nmbrOfLines;
	float lineratio;
	float planeratio;
	std::vector<int> lineInliers;
	std::vector<int> planeInliers;
	cv::Point2d VP;
};
LineReg::LineReg(): nh("~"),it(nh)
{
	pub_det = it.advertise("lines_curv",1);
	pub_can = it.advertise("candidates",1);
	point_pub = nh.advertise<geometry_msgs::PointStamped>("target",3);
//ctor
}

LineReg::~LineReg()
{
//dtor
}
bool LineReg::operator()(cv::UMat  depthImg, cv::UMat  H, cv::UMat CI, std::vector<cv::Rect> rectangles, std::vector<cv::Vec4f>  lines, cv_bridge::CvImagePtr cv_input_ptr, const lsd_slam_viewer::keyframeMsgConstPtr frameMsg, const tum_ardrone::filter_stateConstPtr poseMsg, pcl::PointCloud<pcl::PointXYZ>::Ptr& planeCloud, Eigen::Affine3f& pose)
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
/// \return True if there are candidates detected, false if not

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
	Eigen::Matrix4f trans =Eigen::Matrix4f::Identity();
	Eigen::AngleAxisd rollAngle(poseMsg->roll*CV_PI/180.0, Eigen::Vector3d::UnitZ());
	Eigen::AngleAxisd yawAngle(-poseMsg->yaw*CV_PI/180.0+CV_PI, Eigen::Vector3d::UnitY());
	Eigen::AngleAxisd pitchAngle(poseMsg->pitch*CV_PI/180.0, Eigen::Vector3d::UnitX());

	Eigen::Quaternion<double> q = rollAngle * yawAngle * pitchAngle;
	trans.block<3,3>(0,0) = q.matrix().cast<float>();
	trans(0,3)=poseMsg->x/100.0;
	trans(1,3)=poseMsg->y/100.0;
	trans(2,3)=poseMsg->z/100.0;
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
	for(auto can:candidates)
	{
		pcl::PointCloud<pcl::PointXYZ> temp(*can.cloud,can.planeInliers);
		*planeCloud+=temp;
	}
	if(candidates.size()>0){
		geometry_msgs::PointStamped target;
		target.header.stamp=ros::Time::now();
		target.header.frame_id="tum_base_frontcam";
		for(auto can:candidates)
		{
			if(can.planeInliers.size()<10)
				continue;
			pcl::PointCloud<pcl::PointXYZ> temp(*can.cloud,can.planeInliers);
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

	auto maybeinliers=can.lines;
	ROS_INFO_STREAM("size: "<< maybeinliers.size());
	if(maybeinliers.size()<minLines)
		return;
	random_shuffle(maybeinliers.begin(),maybeinliers.end());
	double TH=200.0f;
	float eps=0.001f;
	int iter(0),maxit(maybeinliers.size());
	std::vector<DepthLine> alsoinliers;
	alsoinliers.reserve(maybeinliers.size());
	unsigned int maxinliers(0);
	while(iter<maxit)
	{
		iter++;
		alsoinliers.clear();
		DepthLine line1=maybeinliers[0];
		DepthLine line2=maybeinliers[1];
		std::rotate(maybeinliers.begin(),maybeinliers.begin()+1,maybeinliers.end());	
		if(line1.rico==line2.rico){
			ROS_INFO("lines are exactly parallel");
			continue;
		}
		cv::Point2d mod;
		mod.x= (line1.constant-line2.constant)/(line2.rico-line1.rico);
		mod.y= mod.x*line1.rico+line1.constant;

		copy_if(maybeinliers.begin(),maybeinliers.end(),back_inserter(alsoinliers),[&mod,&line1,&line2,&TH](DepthLine & line ){
			double x= (line1.constant-line.constant)/(line.rico-line1.rico);
			cv::Point2d inter1(x, line1.rico*x+line1.constant);
			x= (line2.constant-line.constant)/(line.rico-line2.rico);
			cv::Point2d inter2(x, line2.rico*x+line2.constant);
			double dist1 = pow(mod.x-inter1.x,2)+pow(mod.y-inter1.y,2);
			double dist2 = pow(mod.x-inter2.x,2)+pow(mod.y-inter2.y,2);
			return std::min(dist1,dist2)<TH;
		});
		std::vector<cv::Point2d> intersections;
		intersections.reserve((std::pow(alsoinliers.size(),2)-alsoinliers.size())/2.0);
		for(auto it = alsoinliers.begin(); it+1 != alsoinliers.end(); it++)
		{
			std::for_each(it+1,alsoinliers.end(),[&intersections,&it](DepthLine& line){
				double x= (it->constant-line.constant)/(line.rico-it->rico);
				intersections.emplace_back(x, it->rico*x+it->constant);
			});
		}
		cv::Point2d sum=accumulate(intersections.begin(),intersections.end(),cv::Point2d(0.0,0.0),[](cv::Point2d& result, cv::Point2d& point){return result + point;});
		cv::Point2d mean= cv::Point2d(sum.x/intersections.size(), sum.y/intersections.size());
		ROS_INFO_STREAM("intersectionmod: "<< mod.x << ", " << mod.y << "mean: " << mean.x << ", " << mean.y);
		double SSE= accumulate(intersections.begin(),intersections.end(),0.0,[&mean](double result, cv::Point2d point){return result+pow(mean.x-point.x,2)+pow(mean.y-point.y,2);});
		double MSE=SSE/intersections.size();
		if(maybeinliers.size() == alsoinliers.size()){
			if(maybeinliers.size()>minLines)
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
				maxit=static_cast<int>(std::log(eps)/std::log(1.0f-static_cast<float>(maxinliers)/static_cast<float>(maybeinliers.size()))+0.5f);
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

	pcl::PointCloud<pcl::PointXYZ> ekfcloud;
	pcl::transformPointCloud(*can.cloud,ekfcloud,trans);
	pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr model_p (new pcl::SampleConsensusModelPlane<pcl::PointXYZ> (can.cloud, can.lineInliers));
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
	if(!canImg.empty())
	{
		cv::rectangle(canImg, can.rectangle, cv::Scalar(0,255,0));
		std::stringstream data;
		data << "angle: " << can.angle*180.0/CV_PI << "\n"
			 << "nmbr of lines: " << can.nmbrOfLines << "\n"
			<< "ratio: " << static_cast<float>(can.planeInliers.size())/static_cast<float>(can.lineInliers.size()) << "\n"
			<< "MSE: "<< can.bestMSE << "\n"
			<< "sqew: "<< coefficients(1) << "\n";
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
