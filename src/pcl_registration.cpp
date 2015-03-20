#include "pointcloudregistration/pcl_registration.h"
#include <boost/lexical_cast.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/passthrough.h>



PCL_registration::PCL_registration(): cloud (new PointCloud), depth(new PointCloud)
{
	nh=ros::NodeHandle("~");
	pub = nh.advertise<sensor_msgs::PointCloud2> ("points2", 1);
	graph= (new KeyFrameGraph);
	currentCamID=0;
	minX = maxX = minY = maxY=0.0;
	minZ= 0.3;
	maxZ=3;
	ROS_INFO("registration ready");
     //ctor
}

PCL_registration::~PCL_registration()
{
	delete graph;
    //dtor
}
void PCL_registration::getDepthImage(lsd_slam_viewer::keyframeMsgConstPtr msg)
{
	Sophus::Sim3f camToWorld;
	memcpy(camToWorld.data(), msg->camToWorld.data(), 7*sizeof(float));
	Sophus::Sim3f worldToCam=camToWorld.inverse();
	Eigen::Matrix4f transformation = worldToCam.matrix();
	PointCloud::Ptr transformed (new PointCloud);
	pcl::transformPointCloud(*cloud, *transformed, transformation);

	pcl::PassThrough<pcl::PointXYZRGB> passx,passy,passz;
	PointCloud::Ptr outx (new PointCloud);
	PointCloud::Ptr outy (new PointCloud);
	PointCloud::Ptr outz (new PointCloud);
	if(minX==0)
		calcBox(msg);
	passx.setInputCloud (transformed);
	passx.setFilterFieldName ("x");
	passx.setFilterLimits (minX, maxX);
	passx.filter(*outx);

	passy.setInputCloud (outx);
	passy.setFilterFieldName ("y");
	passy.setFilterLimits (minY, maxY);
	passy.filter(*outy);

	passz.setInputCloud (outy);
	passz.setFilterFieldName ("z");
	passz.setFilterLimits (minZ, maxZ);
	passz.filter(*outz);
	depthMutex.lock();
	*depth=*outz;
	depthMutex.unlock();

/*
 for(pcl::PointCloud<pcl::PointXYZRGB>::iterator it = cloud->begin(); it 
!= cloud->end(); it++){ 
    cout << it->x << ", " << it->y << ", " << it->z << endl; 
    }	*/
}
void PCL_registration::calcBox(lsd_slam_viewer::keyframeMsgConstPtr msg)
{
	

	float fxi = 1/msg->fx;
	float fyi = 1/msg->fy;
	float cxi = -msg->cx / msg->fx;
	float cyi = -msg->cy / msg->fy;

	float width = (float) msg->width;
	float height = (float) msg->height;
	minX=(fxi+cxi)*maxZ;
	maxX=(width*fxi + cxi)*maxZ;
	minY=(fyi+cyi)*maxZ;
	maxY=(height*fyi + cyi)*maxZ;
	ROS_INFO_STREAM("box: "<<minX << ", "<< maxX << ", " << minY << ", " << maxY);
}

void PCL_registration::addFrameMsg(lsd_slam_viewer::keyframeMsgConstPtr msg)
{
	meddleMutex.lock();
	if(!msg->isKeyframe)
	{
		if(currentCamID > msg->id)
		{
			printf("detected backward-jump in id (%d to %d), resetting!\n", currentCamID, msg->id);
		}

//		lastAnimTime = lastCamTime = msg->time;
		currentCamID = msg->id;
		getDepthImage(msg);
	}
	else{
		sensor_msgs::PointCloud2::Ptr points = graph->addMsg(msg);
		pub.publish(points);
		graph->refreshPCL();
	}
	meddleMutex.unlock();
	cloud=graph->getPCL();
}
void PCL_registration::addGraphMsg(lsd_slam_viewer::keyframeGraphMsgConstPtr msg)
{
	meddleMutex.lock();
	graph->addGraphMsg(msg);
	meddleMutex.unlock();
}
PointCloud::Ptr PCL_registration::getPCL()
{
	ROS_INFO("retreiving cloud pointer");
	return cloud;
}
PointCloud::Ptr PCL_registration::getDepth()
{
	PointCloud::Ptr tmp(new PointCloud);
	depthMutex.lock();
	*tmp=*depth;
	depthMutex.unlock();
	return tmp;
}
bool PCL_registration::PCLUpdate()
{
	return graph->PCLUpdate();
}
