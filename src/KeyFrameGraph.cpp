

#include "pointcloudregistration/KeyFrameGraph.h"
#include "pointcloudregistration/KeyFrame.h"
#include "ros/package.h"

KeyFrameGraph::KeyFrameGraph():cloud (new PointCloud)
{
	cloudUpdate=false;
}

KeyFrameGraph::~KeyFrameGraph()
{
	for(unsigned int i=0;i<keyframes.size();i++)
		delete keyframes[i];
}

void KeyFrameGraph::refreshPCL()
{
	dataMutex.lock();
	cloud->clear();
	for(std::size_t i=0;i<keyframes.size();i++)
	{
		PointCloud::Ptr tmp=keyframes[i]->getPCL();
		*cloud+=*tmp;
	}
	dataMutex.unlock();
	cloudUpdate=true;
}
PointCloud::Ptr KeyFrameGraph::getPCL()
{
	PointCloud::Ptr retCloud(new PointCloud);
	dataMutex.lock();
	*retCloud = *cloud;
	dataMutex.unlock();
	return retCloud;
}
bool KeyFrameGraph::PCLUpdate()
{
	bool tmp=cloudUpdate;
	cloudUpdate=false;
	return tmp;
}
sensor_msgs::PointCloud2::Ptr KeyFrameGraph::addMsg(lsd_slam_viewer::keyframeMsgConstPtr msg)
{
	dataMutex.lock();
	if(keyframesByID.count(msg->id) == 0)
	{
		KeyFrame* disp = new KeyFrame();
		keyframesByID[msg->id] = disp;
		keyframes.push_back(disp);

	//	printf("added new KF, now there are %d!\n", (int)keyframes.size());
	}

	keyframesByID[msg->id]->setFrom(msg);
	sensor_msgs::PointCloud2::Ptr ros_msg = keyframesByID[msg->id]->getROSMsg(true);
	dataMutex.unlock();
	return ros_msg;
}

void KeyFrameGraph::addGraphMsg(lsd_slam_viewer::keyframeGraphMsgConstPtr msg)
{
	dataMutex.lock();

	constraints.resize(msg->numConstraints);
	assert(msg->constraintsData.size() == sizeof(GraphConstraint)*msg->numConstraints);
	GraphConstraint* constraintsIn = (GraphConstraint*)msg->constraintsData.data();
	for(std::size_t i=0;i<msg->numConstraints;i++)
	{
		constraints[i].err = constraintsIn[i].err;
		constraints[i].from = 0;
		constraints[i].to = 0;

		if(keyframesByID.count(constraintsIn[i].from) != 0)
			constraints[i].from = keyframesByID[constraintsIn[i].from];
//		else
//			printf("ERROR: graph update contains constraints for %d -> %d, but I dont have a frame %d!\n",
//					constraintsIn[i].from,
//					constraintsIn[i].to,
//					constraintsIn[i].from);


		if(keyframesByID.count(constraintsIn[i].to) != 0)
			constraints[i].to = keyframesByID[constraintsIn[i].to];
//		else
//			printf("ERROR: graph update contains constraints for %d -> %d, but I dont have a frame %d!\n",
//					constraintsIn[i].from,
//					constraintsIn[i].to,
//					constraintsIn[i].to);
	}



	GraphFramePose* graphPoses = (GraphFramePose*)msg->frameData.data();
	int numGraphPoses = msg->numFrames;
	assert(msg->frameData.size() == sizeof(GraphFramePose)*msg->numFrames);

	for(int i=0;i<numGraphPoses;i++)
	{
		if(keyframesByID.count(graphPoses[i].id) == 0)
		{
		//	printf("ERROR: graph update contains pose for frame %d, but I dont have a frame %d!\n", graphPoses[i].id, graphPoses[i].id);
		}
		else
			memcpy(keyframesByID[graphPoses[i].id]->camToWorld.data(), graphPoses[i].camToWorld, 7*sizeof(float));
	}

	dataMutex.unlock();

//	printf("graph update: %d constraints, %d poses\n", msg->numConstraints, msg->numFrames);
}
