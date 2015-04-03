
#ifndef KEYFRAMEGRAPH_H_
#define KEYFRAMEGRAPH_H_


#include "lsd_slam_viewer/keyframeGraphMsg.h"
#include "lsd_slam_viewer/keyframeMsg.h"
#include "boost/thread.hpp"
#include "pcl_ros/point_cloud.h"
#include "pcl/point_types.h"
#include "pointcloudregistration/datastructures.h"




class KeyFrameGraph{
public:
	KeyFrameGraph();
	virtual ~KeyFrameGraph();

	void draw();
	//PointCloud::Ptr getPCL();
	bool PCLUpdate();
	//void refreshPCL();
	sensor_msgs::PointCloud2::Ptr addMsg(lsd_slam_viewer::keyframeMsgConstPtr);
	void addGraphMsg(lsd_slam_viewer::keyframeGraphMsgConstPtr);
	std::vector<KeyFrame*> getFrames();

private:
	std::map<int, KeyFrame*> keyframesByID;
	std::vector<KeyFrame*> keyframes;
	std::vector<GraphConstraintPt> constraints;
	//PointCloud::Ptr cloud;
	boost::mutex cloudMutex,graphMutex;
	bool cloudUpdate;

};

#endif /* KEYFRAMEGRAPH_H_ */
