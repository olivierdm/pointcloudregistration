#include "pointcloudregistration/pcl_analyser.h"
#include "pcl/point_types.h"
#include <pcl/common/transforms.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/filters/crop_hull.h>
#include <math.h>
#include <iostream>

PCL_analyser::PCL_analyser(KeyFrameGraph* keyGraph): cloud (new PointCloud), depth(new PointCloud), boundingbox(new PointCloud),nh("~"),it(nh)
{
	pub = it.advertise("depth",1);
	minZ=0.3f;
	maxZ=3.0f;
	wantExit=false;
	data_ready=false;
	camera.setIdentity();
	//create thread
	graph=keyGraph;
	worker = boost::thread(boost::bind(&PCL_analyser::threadLoop,this));
	//ctor
}
PCL_analyser::~PCL_analyser()
{
	//destroy thread
	{
		boost::mutex::scoped_lock lock(frameMutex);
		wantExit=true;
		newData.notify_one();
	}
	worker.join();
	//dtor
}
void PCL_analyser::getDepthImage()
{
	boost::mutex::scoped_lock lock(framesMtx);

	std::vector<KeyFrame*> keyframes = graph->getFrames();
	ROS_INFO_STREAM("opencv image height: " << height << ", width: "<< width);

	Eigen::Vector3f point2de(0.0f,0.0f,1.0f);//extended coordinates
	Eigen::Vector3f point3dT(0.0f,0.0f,0.0f);//extended coordinates
	Eigen::Vector4f point3de(0.0f,0.0f,0.0f,1.0f);
	PointCloud::Ptr roi (new PointCloud);
	PointCloud::Ptr surface_hull (new PointCloud);
    	pcl::ConvexHull<pcl::PointXYZRGB> hull;
	hull.setDimension(3);
	pcl::CropHull<pcl::PointXYZRGB> bb_filter;
	bb_filter.setDim(3);
	int x,y,num;
	num=0;
	std::vector<pcl::Vertices> polygons;
	depthImg.setTo(0.0f);
	for(std::size_t i=0;i<keyframes.size();i++)
	{
	//get boundingbox in analysed keyframe
	setProjection(camToWorld.inverse(),keyframes[i]->camToWorld);
	ROS_DEBUG("transform bounds");	
	pcl::transformPointCloud(*boundingbox, *roi, soph);
	ROS_DEBUG("construct hull");
	hull.setInputCloud(roi);
	hull.reconstruct(*surface_hull, polygons);
	ROS_INFO("initialize filter");

	bb_filter.setHullIndices(polygons);
	bb_filter.setHullCloud(roi);
	//get the cloud and lock it
	bb_filter.setInputCloud(keyframes[i]->getPCL());
	bb_filter.filter(*depth);
	//unlock cloud
	keyframes[i]->release();


	ROS_INFO_STREAM("iterating " << i+1 << " the time, through cloud "<<keyframes[i]->id <<"points: "<< depth->size());
	for(PointCloud::iterator it = depth->begin(); it != depth->end(); it++){ 
		point3de(0)=it->x;
		point3de(1)=it->y;
		point3de(2)=it->z;
		//calculation may be further optimized as only z is needed in camera perspective
		point3dT=rotTrans*point3de;//calc point in camera perspective
		point2de=projection*point3de; //calc projected coordinates
		x=static_cast<int> (point2de(0)/point2de(2)+0.5f);
		y=static_cast<int> (point2de(1)/point2de(2)+0.5f);
		if(x<0||x>width||y<0||y>height)
			continue;
		depthImg.at<float>(y,x)= point3dT(2)*1000.0f;
		num++;		
    	}

	depth->clear();
	roi->clear();
	polygons.clear();
	surface_hull->clear();
	}
	ROS_INFO("converting message");
	msg = cv_bridge::CvImage(header, "mono16", depthImg.clone()).toImageMsg();
	ROS_INFO_STREAM("sending message, "<< num << " points");
	pub.publish(msg);
}
void PCL_analyser::process(lsd_slam_viewer::keyframeMsgConstPtr msg)
{
	//accept the new data
	{
		boost::mutex::scoped_lock lock(frameMutex);
		memcpy(camToWorld.data(), msg->camToWorld.data(), 7*sizeof(float));
		data_ready=true;
		if(boundingbox->empty())
			calcBox(msg);
		//copy header
		header=msg->header;
	}
	//notify thread
	newData.notify_one();
}
void PCL_analyser::threadLoop()
{
	ROS_INFO("pcl analyser thread started");
	while(true)
	{
		boost::mutex::scoped_lock lock(frameMutex);
		while(!data_ready)//check if new message passed
		{
			newData.wait(lock);
		}
		data_ready=false;
		if(wantExit)
			return;
		ROS_INFO("starting detection loop");
		getDepthImage();
		calcCurvature();
	}
}
void PCL_analyser::calcCurvature()
{
	ROS_INFO("dummy calc curvature");
}

void PCL_analyser::calcBox(lsd_slam_viewer::keyframeMsgConstPtr msg)
{
	

	float fxi = 1.0f/msg->fx;
	float fyi = 1.0f/msg->fy;
	float cxi = -msg->cx / msg->fx;
	float cyi = -msg->cy / msg->fy;
	setCamera(msg->fx,msg->fy,msg->cx,msg->cy);
	float minx,minX,maxx,maxX,miny,minY,maxy,maxY;
	width=msg->width;
	height=msg->height;
	depthImg=cv::Mat(height,width,CV_32F,0.0f);
	float widthf = static_cast<float> (width);
	float heightf = static_cast<float> (height);
	minX=(fxi+cxi)*maxZ;
	maxX=(widthf*fxi + cxi)*maxZ;

	minx=(fxi+cxi)*minZ;
	maxx=(widthf*fxi + cxi)*minZ;

	minY=(fyi+cyi)*maxZ;
	maxY=(heightf*fyi + cyi)*maxZ;

	miny=(fyi+cyi)*maxZ;
	maxY=(heightf*fyi + cyi)*maxZ;
	pcl::PointXYZRGB pt;

	pt.x=minX;
	pt.y=maxY;
	pt.z=maxZ;
	boundingbox->push_back(pt);
	pt.x=maxX;
	boundingbox->push_back(pt);
	pt.y=minY;
	boundingbox->push_back(pt);
	pt.x=minX;
	boundingbox->push_back(pt);
	pt.x=minx;
	pt.y=maxy;
	pt.z=minZ;
	boundingbox->push_back(pt);
	pt.x=maxx;
	boundingbox->push_back(pt);
	pt.y=miny;
	boundingbox->push_back(pt);
	pt.x=minx;
	boundingbox->push_back(pt);
}
void PCL_analyser::setCamera(float fx, float fy, float cx, float cy)
{
	camera(0,0)=fx;
	camera(0,2)=cx;
	camera(1,1)=fy;
	camera(1,2)=cy;
	ROS_INFO_STREAM("camera:\n"<< camera);
}
void PCL_analyser::setProjection(Sophus::Sim3f worldToCam,Sophus::Sim3f kfToWorld)
{
	soph = worldToCam.matrix()*kfToWorld.matrix();
	rotTrans =soph.block<3,4>(0,0);
	projection= camera*rotTrans;
	soph = kfToWorld.inverse().matrix()*worldToCam.inverse().matrix();
}
bool PCL_analyser::ready()
{
	boost::mutex::scoped_lock lock(frameMutex, boost::try_to_lock);
	if(lock)
	{
		return true;
	}else{
		return false;
	}
}
