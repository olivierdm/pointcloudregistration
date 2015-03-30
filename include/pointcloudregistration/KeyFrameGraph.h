
#ifndef KEYFRAMEGRAPHDISPLAY_H_
#define KEYFRAMEGRAPHDISPLAY_H_


#include "lsd_slam_viewer/keyframeGraphMsg.h"
#include "lsd_slam_viewer/keyframeMsg.h"
#include "boost/thread.hpp"
#include "pcl_ros/point_cloud.h"
#include "pcl/point_types.h"

typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;
class KeyFrame;


struct GraphConstraint
{
	int from;
	int to;
	float err;
};


struct GraphConstraintPt
{
	KeyFrame* from;
	KeyFrame* to;
	float err;
};

struct GraphFramePose
{
	int id;
	float camToWorld[7];
};


class KeyFrameGraph{
public:
	KeyFrameGraph();
	virtual ~KeyFrameGraph();

	void draw();
	PointCloud::Ptr getPCL();
	bool PCLUpdate();
	void refreshPCL();
	sensor_msgs::PointCloud2::Ptr addMsg(lsd_slam_viewer::keyframeMsgConstPtr);
	void addGraphMsg(lsd_slam_viewer::keyframeGraphMsgConstPtr);
	std::vector<KeyFrame*> getFrames();

private:
	std::map<int, KeyFrame*> keyframesByID;
	std::vector<KeyFrame*> keyframes;
	std::vector<GraphConstraintPt> constraints;
	PointCloud::Ptr cloud;
	boost::mutex cloudMutex,graphMutex;
	bool cloudUpdate;

};

#endif /* KEYFRAMEGRAPHDISPLAY_H_ */
