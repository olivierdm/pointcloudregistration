#include "pointcloudregistration/linereg.h"
#include <pcl/filters/extract_indices.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_line.h>

LineReg::LineReg()
{
//ctor
}

LineReg::~LineReg()
{
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
		while(!rectangles_ready||!curvature_ready||!lines_ready)//check if new message passed
		{
			newData.wait(lock);
		}
		rectangles_ready=false;
		curvature_ready=false;
		lines_ready=false;
		if(wantExit)
			return;
		get3DLines();
	}
}
void LineReg::get3DLines()
{
 std::vector<int> inliers;
 pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  // created RandomSampleConsensus object and compute the appropriated model
  pcl::SampleConsensusModelLine<pcl::PointXYZ>::Ptr
    model_l (new pcl::SampleConsensusModelLine<pcl::PointXYZ> (cloud));

    pcl::RandomSampleConsensus<pcl::PointXYZ> ransac (model_l);
    ransac.setDistanceThreshold (.01);
    ransac.computeModel();
    ransac.getInliers(inliers);
}
void LineReg::process(std::vector<cv::Rect> rectangles,std::vector<cv::Vec4f> lines, std::vector<float> quality)//objectdet,lsd
{
///
/// \brief  Accepts the data of an image and copies the neccesary data to the adequate private variables.
/// @param[in] msg an image
/// 
	boost::mutex::scoped_lock lock(dataMutex);
	rectangles_ready=true;
	//notify thread
	newData.notify_one();
}
void LineReg::process(cv::UMat CI,cv::UMat depthImg)//depthproc
{
///
/// \brief  Accepts the data of an image and copies the neccesary data to the adequate private variables.
/// @param[in] msg an image
/// 
	boost::mutex::scoped_lock lock(dataMutex);
	curvature_ready=true;
	//notify thread
	newData.notify_one();
}
