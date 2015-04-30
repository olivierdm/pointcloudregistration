#include "pointcloudregistration/KeyFrameGraph.h"
#include "pointcloudregistration/KeyFrame.h"
#include "ros/package.h"

KeyFrameGraph::KeyFrameGraph(): cloudUpdate(false)
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

}

bool KeyFrameGraph::PCLUpdate()
{
	boost::mutex::scoped_lock lock(cloudMutex);
	bool tmp=cloudUpdate;
	cloudUpdate=false;
	return tmp;
}
void KeyFrameGraph::addMsg(lsd_slam_viewer::keyframeMsgConstPtr msg)
{
/** 
\brief Adds a new keyframe to the graph
@param[in] msg keyframe message
*/ 
	boost::mutex::scoped_lock lock(graphMutex);
	if(keyframesByID.count(msg->id) == 0)
	{
		keyframesByID[msg->id] = std::make_shared<KeyFrame>();


	//	printf("added new KF, now there are %d!\n", (int)keyframes.size());
	}

	keyframesByID[msg->id]->setFrom(msg);
	boost::mutex::scoped_lock lockc(cloudMutex);
	cloudUpdate=true;
}
void KeyFrameGraph::reset()
{
	boost::mutex::scoped_lock lock(graphMutex);
	keyframesByID.clear();
}
void KeyFrameGraph::addGraphMsg(lsd_slam_viewer::keyframeGraphMsgConstPtr msg)
{
/**
\brief Updates the graph constraints and updates the newly calculated poses.
@param[in] msg keyframe graph message
*/

	GraphFramePose* graphPoses = (GraphFramePose*)msg->frameData.data();
	int numGraphPoses = msg->numFrames;
	assert(msg->frameData.size() == sizeof(GraphFramePose)*msg->numFrames);
	boost::mutex::scoped_lock lock(graphMutex);
	for(int i=0;i<numGraphPoses;i++)
	{
		if(keyframesByID.count(graphPoses[i].id) == 0)
		{
		//	printf("ERROR: graph update contains pose for frame %d, but I dont have a frame %d!\n", graphPoses[i].id, graphPoses[i].id);
		}
		else
			memcpy(keyframesByID[graphPoses[i].id]->camToWorld.data(), graphPoses[i].camToWorld, 7*sizeof(float));
	}
	boost::mutex::scoped_lock lockc(cloudMutex);
	cloudUpdate=true;
}
std::vector<std::shared_ptr<KeyFrame>> KeyFrameGraph::getFrames()
{
/** \brief Returns an array of pointers to the current set of keyframes. */
	boost::mutex::scoped_lock lock(graphMutex);
	std::vector<std::shared_ptr<KeyFrame>> keyframes;
	keyframes.reserve(keyframesByID.size());
	transform(keyframesByID.begin(),keyframesByID.end(),std::back_inserter(keyframes),[](std::pair<const int, std::shared_ptr<KeyFrame> >& pair){return pair.second;});
	return keyframes;
}
