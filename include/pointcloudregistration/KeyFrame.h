#include <Eigen/Core>
#include "lsd_slam_viewer/keyframeMsg.h"
#include "sophus/sim3.hpp"
#include "pcl_conversions/pcl_conversions.h"
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;
struct InputPointDense
{
	float idepth;
	float idepth_var;
	unsigned char color[4];
};

// stores a pointcloud associated to a Keyframe.
class KeyFrame
{

public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	KeyFrame();
	~KeyFrame();


	void setFrom(lsd_slam_viewer::keyframeMsgConstPtr msg);
	sensor_msgs::PointCloud2::Ptr getROSMsg(bool);
	PointCloud::Ptr getPCL();
	int id;
	double time;

	int totalPoints, displayedPoints;

	// camera pose
	// may be updated by kf-graph.
	Sophus::Sim3f camToWorld;

private:
	// camera parameter
	// fixed.
	float fx,fy,cx,cy;
	float fxi,fyi,cxi,cyi;
	int width, height;
	void refreshPCL();
	bool cloudValid;

	float my_scaledTH, my_absTH, my_scale;
	int my_minNearSupport;
	int my_sparsifyFactor;


	// pointcloud data & respective buffer
	PointCloud::Ptr cloud, cloudLocal;
	InputPointDense* originalInput;
};

