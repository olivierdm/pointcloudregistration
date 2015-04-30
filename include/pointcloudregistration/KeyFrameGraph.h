
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
	void addMsg(lsd_slam_viewer::keyframeMsgConstPtr);
	void addGraphMsg(lsd_slam_viewer::keyframeGraphMsgConstPtr);
	std::vector<std::shared_ptr<KeyFrame>> getFrames();
	void reset();

private:
	std::map<int, std::shared_ptr<KeyFrame>> keyframesByID;
	//std::vector<std::shared_ptr<KeyFrame>> keyframes;
	boost::mutex cloudMutex,graphMutex;
	bool cloudUpdate;

};

#endif /* KEYFRAMEGRAPH_H_ */
