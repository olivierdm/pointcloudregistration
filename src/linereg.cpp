#include "pointcloudregistration/linereg.h"
#include "pointcloudregistration/settings.h"
#include <pcl/filters/extract_indices.h>
#include <sensor_msgs/image_encodings.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_line.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include "pointcloudregistration/pcl_registration.h"
#include "tbb/parallel_for_each.h"
#include <cmath>        // std::abs
#include <iterator>     // std::back_inserter

struct DepthLine
{
DepthLine(cv::Vec4f & lin):line(lin),curvature(0.0),rico(6.0f)
{
	//ctor
}
//DepthLine():curvature(0.0){}
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
	int nmbrOfLines;
	std::vector<int> lineInliers;
	std::vector<int> planeInliers;
};
LineReg::LineReg(PCL_registration* reg):registrar(reg),nh("~"),it(nh),cv_debug_ptr(new cv_bridge::CvImage),wantExit(false),rectangles_ready(false),curvature_ready(false)
{
	pub_det = it.advertise("lines_curv",1);
	cv_debug_ptr->encoding="bgr8";
	cv_debug_ptr->header.frame_id="line_segment";
	//worker = boost::thread(boost::bind(&LineReg::threadLoop,this));
//ctor
}

LineReg::~LineReg()
{
	{
	boost::mutex::scoped_lock lock(dataMutex);
	wantExit=true;
	newData.notify_all();
	}
	//worker.join();
//dtor
}
void LineReg::operator()(cv::UMat & depthImg, cv::UMat &  H, cv::UMat & CI, std::vector<cv::Rect> & rectangles, std::vector<cv::Vec4f> &  lines, const sensor_msgs::ImageConstPtr& imgMsg, const lsd_slam_viewer::keyframeMsgConstPtr &frameMsg, const tum_ardrone::filter_stateConstPtr &poseMsg)
{
	boost::mutex::scoped_lock lock(dataMutex);
	candidates.clear();
	depthLines.clear();
	candidates.reserve(rectangles.size());
	for_each(rectangles.begin(),rectangles.end(),[this](cv::Rect & rectangle){candidates.push_back(Candidate(rectangle));});
	depthLines.reserve(lines.size());
	for_each(lines.begin(),lines.end(),[this](cv::Vec4f & line ){depthLines.push_back(DepthLine(line));});
	//copy the msg to a cv::Mat instance
	try
	{
		cv_debug_ptr = cv_bridge::toCvCopy(imgMsg, sensor_msgs::image_encodings::BGR8);
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}
	cv::Mat curv_weight, meanCurvature(H.getMat(cv::ACCESS_READ));
	cv::add(CI.mul(0.95f),0.05f,curv_weight);
	float fxi=1.0f/frameMsg->fx;
	float fyi=1.0f/frameMsg->fy;
	float cxi=-frameMsg->cx * fxi;
	float cyi=-frameMsg->cy *fyi;
tbb::parallel_for_each(candidates.begin(),candidates.end(),[&](Candidate & can){
		getParallelLines(can);
		if(can.nmbrOfLines==0)
			return;
		get3DLines(can, curv_weight, meanCurvature, fxi, fyi, cxi, cyi);
		getPlane(can);
		ROS_INFO("process candidate");});
	if(pub_det.getNumSubscribers() != 0)
		pub_det.publish(cv_debug_ptr->toImageMsg());
}
/*void LineReg::threadLoop()
{
///
/// \brief This method is started in a different thread and waits for data. Upon reception of new data the methods
///Vision::getLines() and Vision::detect() are called.
///
	ROS_INFO_STREAM("line regression constructed");
	while(true)
	{
		boost::mutex::scoped_lock lock(dataMutex);
		while(!rectangles_ready||!curvature_ready)//check if new message passed
		{
			newData.wait(lock);
		}
		rectangles_ready=false;
		curvature_ready=false;
		if(wantExit)
			return;
tbb::parallel_for_each(candidates.begin(),candidates.end(),[this](Candidate & can){
		getParallelLines(can);
		if(can.nmbrOfLines==0)
			return;
		get3DLines(can);
		getPlane(can);
		ROS_INFO("process candidate");});
	if(pub_det.getNumSubscribers() != 0)
		pub_det.publish(cv_debug_ptr->toImageMsg());

	//candidates.erase(remove_if(candidates.begin(),candidates.end(),[](Candidate & can){return can.nmbrOfLines==0;}));
	}
}*/
void LineReg::getParallelLines(Candidate & can)
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
			if(maybeinliers.size() == alsoinliers.size()){
				can.nmbrOfLines=alsoinliers.size();
				break;
			}
			float sum = accumulate(alsoinliers.begin(),alsoinliers.end(),0.0f,[](float result, DepthLine& line){return result+line.rico;});
			float modelrico = sum/alsoinliers.size();
			float SSE= accumulate(alsoinliers.begin(),alsoinliers.end(),0.0f,[&modelrico](float result, DepthLine line){return result+pow(line.rico-modelrico,2);});
			float MSE=SSE/alsoinliers.size();
			if(alsoinliers.size()>6 )
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
void LineReg::get3DLines(Candidate & can, cv::Mat & curv_weight,cv::Mat & meanCurvature, float & fxi, float & fyi, float & cxi, float & cyi)
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
void LineReg::getPlane(Candidate & can)
{
		if(can.lineInliers.empty())
			return;
		pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr model_p (new pcl::SampleConsensusModelPlane<pcl::PointXYZ> (can.cloud, can.lineInliers));

		pcl::RandomSampleConsensus<pcl::PointXYZ> ransac (model_p);
		ransac.setDistanceThreshold (.01);
		ransac.computeModel();
		ransac.getInliers(can.planeInliers);
		//pcl::PointCloud<pcl::PointXYZ>::Ptr plane(new pcl::PointCloud<pcl::PointXYZ>(*cloud,planeInliers));
	if(pub_det.getNumSubscribers() != 0)
	{
		for(size_t i=0 ; i < can.lines.size();i++)
		{
			cv::Vec4f line =can.lines[i].line;
			cv::Scalar color(0,255*can.lines[i].curvature,255*(1.0f-can.lines[i].curvature));
			cv::line(cv_debug_ptr->image,cv::Point2f(line[0],line[1]),cv::Point2f(line[2],line[3]),color);
		}
	}
}

/*void LineReg::process(std::vector<cv::Rect> rectangles,std::vector<cv::Vec4f> lines, std::vector<float> quality,cv::Mat original)//objectdet,lsd
{
///
/// \brief  Accepts the data of an image and copies the neccesary data to the adequate private variables.
/// @param[in] good_lines set of lines that are detected on the visual image
/// @param[in] quality vector indicating the quality of each line 
///
	boost::mutex::scoped_lock lock(dataMutex);
	ROS_INFO_STREAM("received vision data");
	lineQuality=quality;
	candidates.clear();
	depthLines.clear();
	candidates.reserve(rectangles.size());
	for_each(rectangles.begin(),rectangles.end(),[this](cv::Rect & rectangle){candidates.push_back(Candidate(rectangle));});
	depthLines.reserve(lines.size());
	for_each(lines.begin(),lines.end(),[this](cv::Vec4f & line ){depthLines.push_back(DepthLine(line));});
	cv_debug_ptr->image=original;
	rectangles_ready=true;
	//notify thread
	newData.notify_one();
}
void LineReg::process(cv::Mat CI,cv::Mat depthImgf,cv::Mat H, cv::Vec4f camera, const Eigen::Affine3f & pose )//depthproc
{
///
/// \brief  Accepts the data of an image and copies the neccesary data to the adequate private variables.
/// @param[in] CI matrix containing the curvature in each point
/// @param[in] deptImgf filtered depth image
/// @param[in] vector containing the intrinsic camera parameters
/// 
	boost::mutex::scoped_lock lock(dataMutex);
	ROS_INFO_STREAM("received depth data");
	fxi=1.0f/camera[0];
	fyi=1.0f/camera[1];
	cxi=-camera[2]*fxi;
	cyi=-camera[3]*fyi;
	depthImg=depthImgf;
	meanCurvature = H;
	cv::add(CI.mul(0.95f),0.05f,curv_weight);
	campose=pose;
	curvature_ready=true;
	//notify thread
	newData.notify_one();
}*/
/*bool LineReg::SegmentOutsideRectangles(cv::Vec4f &line)
{
	for(auto j=rectangles.begin();j!=rectangles.end();j++)
	{
		if(SegmentIntersectRectangle(*j, line))
			return false;
	}
	return true;
}*/

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
