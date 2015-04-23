#include "pointcloudregistration/linereg.h"
#include "pointcloudregistration/settings.h"
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_line.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include "pointcloudregistration/pcl_registration.h"
#include <cmath>        // std::abs
struct DepthLine
{
DepthLine(cv::Vec4f & lin):line(lin){}
cv::Vec4f line;
std::vector<int> points;
std::vector<int> inliers;
double curvature;
bool convex;
};
struct Candidate
{
	Candidate(cv::Rect & rect): rectangle(rect){}
	cv::Rect rectangle;
	std::vector<DepthLine> lines;
};
LineReg::LineReg(PCL_registration* reg):registrar(reg),nh("~"),it(nh),cloud(new pcl::PointCloud<pcl::PointXYZ>),cv_debug_ptr(new cv_bridge::CvImage),wantExit(false),rectangles_ready(false),curvature_ready(false)
{
	pub_det = it.advertise("lines_curv",1);
	cv_debug_ptr->encoding="bgr8";
	cv_debug_ptr->header.frame_id="line_segment";
	worker = boost::thread(boost::bind(&LineReg::threadLoop,this));
//ctor
}

LineReg::~LineReg()
{
	{
	boost::mutex::scoped_lock lock(dataMutex);
	wantExit=true;
	newData.notify_all();
	}
	worker.join();
	cloud.reset();
//dtor
}

void LineReg::threadLoop()
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
		get3DLines();
	}
}
void LineReg::get3DLines()
{
ROS_INFO("getting lines");
cloud->clear();
 std::vector<int> inliers;

//get points from lines and depthImage
//lines.erase(remove_if(lines.begin(),lines.end(),[this](cv::Vec4f& line){return SegmentOutsideRectangles(line);}),lines.end());
	/*for(size_t i = 0; i < lines.size();i++)
	{
		double curvature=0.0;
		double H = 0.0;		
		cv::LineIterator it(depthImg,cv::Point(lines[i][0],lines[i][1]),cv::Point(lines[i][2],lines[i][3]));
		double nmbr=0.0;
		for(int j = 0; j < it.count; j++,++it)
		{
			cv::Point linePt=it.pos();
			float depth=depthImg.at<float>(linePt);
			if(std::abs(maxZ-depth)<0.5f)
				continue;
			nmbr++;
			curvature+=curv_weight.at<double>(linePt);
			H+=meanCurvature.at<double>(linePt);
			pcl::PointXYZ point;
		    	point.x=(linePt.x*fxi + cxi)*depth;
		    	point.y=(linePt.y*fyi + cyi)*depth;
		    	point.z=depth;
			cloud->push_back(point);
			depthLines[i].points.push_back(static_cast<int>(cloud->size()));
		}
		depthLines[i].convex=0.0<H;
		depthLines[i].curvature=curvature/nmbr;
		depthLines[i].lineId=i;

	}*/
	for(auto it=candidates.begin();it != candidates.end(); it++)
	{
		remove_copy_if (depthLines.begin(),depthLines.end(),it->lines.begin(),[&](DepthLine& line){return SegmentOutsideRectangle(it->rectangle,line.line);});
	}
	std::vector<int> lineInliers;
	for(size_t i = 0; i < depthLines.size();i++)
	{
		if(depthLines[i].points.empty())
			continue;
		// created RandomSampleConsensus object and compute the appropriated model
		pcl::SampleConsensusModelLine<pcl::PointXYZ>::Ptr model_l (new pcl::SampleConsensusModelLine<pcl::PointXYZ> (cloud,depthLines[i].points));

		pcl::RandomSampleConsensus<pcl::PointXYZ> ransac (model_l);
		ransac.setDistanceThreshold (.01);
		ransac.computeModel();
		ransac.getInliers(depthLines[i].inliers);
		lineInliers.insert(lineInliers.end(),depthLines[i].inliers.begin(),depthLines[i].inliers.end());
//ROS_INFO_STREAM("ratio"<< static_cast<float>(depthLines[i].inliers.size())/static_cast<float>(depthLines[i].points.size()) << " mean curvature: "<< depthLines[i].curvature);
	}
		std::vector<int> planeInliers;
		pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr model_p (new pcl::SampleConsensusModelPlane<pcl::PointXYZ> (cloud,lineInliers));

		pcl::RandomSampleConsensus<pcl::PointXYZ> ransac (model_p);
		ransac.setDistanceThreshold (.01);
		ransac.computeModel();
		ransac.getInliers(planeInliers);
		pcl::PointCloud<pcl::PointXYZ>::Ptr plane(new pcl::PointCloud<pcl::PointXYZ>(*cloud,planeInliers));
		registrar->drawPlane(plane,campose);
	if (pub_det.getNumSubscribers() != 0)
	{
		/*for(size_t i=0 ; i < depthLines.size();i++)
		{
			cv::Vec4f line =lines[depthLines[i].lineId];
			cv::Scalar color(0,255*depthLines[i].curvature,255*(1.0f-depthLines[i].curvature));
			cv::line(cv_debug_ptr->image,cv::Point2f(line[0],line[1]),cv::Point2f(line[2],line[3]),color);
		}
		pub_det.publish(cv_debug_ptr->toImageMsg());*/
	}
}

void LineReg::process(std::vector<cv::Rect> rectangles,std::vector<cv::Vec4f> good_lines, std::vector<float> quality,cv::Mat original)//objectdet,lsd
{
///
/// \brief  Accepts the data of an image and copies the neccesary data to the adequate private variables.
/// @param[in] good_lines set of lines that are detected on the visual image
/// @param[in] quality vector indicating the quality of each line 
///
	boost::mutex::scoped_lock lock(dataMutex);
	ROS_INFO_STREAM("received vision data");
	lines=good_lines;
	lineQuality=quality;
	candidates.clear();
	candidates.reserve(rectangles.size());
	for_each(rectangles.begin(),rectangles.end(),[this](cv::Rect & rectangle){candidates.push_back(Candidate(rectangle));});
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
}
/*bool LineReg::SegmentOutsideRectangles(cv::Vec4f &line)
{
	for(auto j=rectangles.begin();j!=rectangles.end();j++)
	{
		if(SegmentIntersectRectangle(*j, line))
			return false;
	}
	return true;
}*/
bool LineReg::SegmentOutsideRectangle(cv::Rect& rectangle, cv::Vec4f& line)
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
      return true;
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
      return true;
    }

    return false;
  }
