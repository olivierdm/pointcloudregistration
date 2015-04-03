

#include "pointcloudregistration/KeyFrameGraph.h"
#include "pointcloudregistration/KeyFrame.h"
#include "ros/package.h"

KeyFrameGraph::KeyFrameGraph(): cloudUpdate(false)//,cloud (new PointCloud)
{
///
/// \brief default constructor
///
}

KeyFrameGraph::~KeyFrameGraph()
{
///
/// \brief The default destructor cleans up the keyframes in the graph
///
	for(std::size_t i=0;i<keyframes.size();i++)
		delete keyframes[i];
}
/*
void KeyFrameGraph::refreshPCL()
{

	cloud->clear();
	for(std::size_t i=0;i<keyframes.size();i++)
	{
		PointCloud::Ptr tmp=keyframes[i]->getPCL();
		*cloud+=*tmp;
		keyframes[i]->release();
	}
	cloudUpdate=true;
}
PointCloud::Ptr KeyFrameGraph::getPCL()
{
	PointCloud::Ptr retCloud(new PointCloud);
	boost::mutex::scoped_lock lock(cloudMutex);
	*retCloud = *cloud;
	return retCloud;
}*/
bool KeyFrameGraph::PCLUpdate()
{
	boost::mutex::scoped_lock lock(cloudMutex);
	bool tmp=cloudUpdate;
	cloudUpdate=false;
	return tmp;
}
sensor_msgs::PointCloud2::Ptr KeyFrameGraph::addMsg(lsd_slam_viewer::keyframeMsgConstPtr msg)
{
/** 
\brief Adds a new keyframe to the graph and returns a ros message
@param[in] msg keyframe message
*/ 
	{
	boost::mutex::scoped_lock lock(graphMutex);
	if(keyframesByID.count(msg->id) == 0)
	{
		KeyFrame* disp = new KeyFrame();
		keyframesByID[msg->id] = disp;
		keyframes.push_back(disp);

	//	printf("added new KF, now there are %d!\n", (int)keyframes.size());
	}

	keyframesByID[msg->id]->setFrom(msg);
	}
	sensor_msgs::PointCloud2::Ptr ros_msg = keyframesByID[msg->id]->getROSMsg();
	return ros_msg;
}

void KeyFrameGraph::addGraphMsg(lsd_slam_viewer::keyframeGraphMsgConstPtr msg)
{
/**
\brief Updates the graph constraints and updates the newly calculated poses.
@param[in] msg keyframe graph message
*/
	boost::mutex::scoped_lock lock(graphMutex);
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


//	printf("graph update: %d constraints, %d poses\n", msg->numConstraints, msg->numFrames);
}
std::vector<KeyFrame*> KeyFrameGraph::getFrames()
{
/** \brief Returns an array of pointers to the current set of keyframes. */
	boost::mutex::scoped_lock lock(graphMutex);
	return keyframes;
}
