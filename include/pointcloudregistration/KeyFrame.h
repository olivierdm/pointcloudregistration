#include <Eigen/Core>
#include "lsd_slam_viewer/keyframeMsg.h"
#include "sophus/sim3.hpp"
#include "pcl/common/common.h"
#include "boost/thread.hpp"
#include "pointcloudregistration/datastructures.h"



// stores a pointcloud associated to a Keyframe.
class KeyFrame
{

public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	KeyFrame();
	~KeyFrame();


	void setFrom(lsd_slam_viewer::keyframeMsgConstPtr msg);
	//sensor_msgs::PointCloud2::Ptr getROSMsg();
	PointCloud::Ptr getPCL();
	int id;
	double time;
	void release();
	int totalPoints, displayedPoints;
	/// camera pose, may be updated by kf-graph.
	Sophus::Sim3f camToWorld; 

private:
	float fx; ///< focal length in x direction
	float fy; ///< focal length in y direction
	float cx; ///< first ordinate of the camera's principal point
	float cy; ///< second ordinate of th camera's principal point
	float fxi,fyi,cxi,cyi;
	int width;///< image width
	int height;///< image height
	void refreshPCL();
	boost::mutex cloudMutex;

	float my_scaledTH, my_absTH, my_scale;
	int my_minNearSupport;
	int my_sparsifyFactor;


	/// pointcloud data in pcl format, stays always in the local coordinates of the keyframe
	PointCloud::Ptr cloud;
	/// datastructure for casting the incomming message
	InputPointDense* originalInput;
};

