#include "pointcloudregistration/pcl_analyser.h"
#include "pcl/point_types.h"
#include <pcl/common/transforms.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/filters/crop_hull.h>
#include <math.h>
PCL_analyser::PCL_analyser(): cloud (new PointCloud), depth(new PointCloud), boundingbox(new PointCloud),nh("~"),it(nh)
{
	pub = it.advertise("depth",1);
	minZ=0.3f;
	maxZ=3.0f;
	wantExit=false;
	//create thread
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
	PointCloud::Ptr roi (new PointCloud);
	PointCloud::Ptr depthWorld(new PointCloud);

	pcl::transformPointCloud(*boundingbox, *roi, camToWorld.matrix());

    	pcl::ConvexHull<pcl::PointXYZRGB> hull;
	hull.setInputCloud(roi);
	hull.setDimension(3);
	std::vector<pcl::Vertices> polygons;

	PointCloud::Ptr surface_hull (new PointCloud);
	hull.reconstruct(*surface_hull, polygons);
	pcl::CropHull<pcl::PointXYZRGB> bb_filter;

	bb_filter.setDim(3);
	bb_filter.setInputCloud(cloud);
	bb_filter.setHullIndices(polygons);
	bb_filter.setHullCloud(boundingbox);
	bb_filter.filter(*depthWorld);

	setProjection(camToWorld.inverse());

	cv::Mat depthImg(width,height,CV_32F,cv::Scalar(0));
	Eigen::Vector3f point2de(0.0f,0.0f,1.0f);//extended coordinates
	Eigen::Vector3f point3dT(0.0f,0.0f,0.0f);//extended coordinates
	Eigen::Vector4f point3de(0.0f,0.0f,0.0f,1.0f);
	int x,y;
	float depth;
	for(PointCloud::iterator it = cloud->begin(); it != cloud->end(); it++){ 
		point3de(0)=it->x;
		point3de(1)=it->y;
		point3de(2)=it->z;
		//calculation may be further optimized as only z is needed in camera perspective
		point3dT=rotTrans*point3de;//calc point in camera perspective
		point2de=projection*point3de; //calc projected coordinates
		x=(int) round(point2de(0)/point2de(2));
		y=(int) round(point2de(1)/point2de(2));
		if(x<0||x>width||y<0||y>height)
			continue;
		depthImg.at<float>(x,y)=point3dT(2);		
    	}
	sensor_msgs::ImagePtr msg = cv_bridge::CvImage(msg->header, "mono16", depthImg).toImageMsg();
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
	}
	//notify thread
	newData.notify_one();
}
void PCL_analyser::threadLoop()
{
	ROS_INFO("registration ready");
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
		getDepthImage();
		calcCurvature();
	}
}
void PCL_analyser::calcCurvature()
{

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
	float widthf = (float) width;
	float heightf = (float) height;
	minX=(fxi+cxi)*maxZ;
	maxX=(widthf*fxi + cxi)*maxZ;

	minx=(fxi+cxi)*minZ;
	maxx=(widthf*fxi + cxi)*minZ;

	minY=(fyi+cyi)*maxZ;
	maxY=(heightf*fyi + cyi)*maxZ;

	miny=(fyi+cyi)*maxZ;
	maxY=(heightf*fyi + cyi)*maxZ;
	PointCloud::Ptr boundingbox(new PointCloud);
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
	camera(0,1)=0.0f;
	camera(0,2)=cx;
	camera(1,0)=0.0f;
	camera(1,1)=fy;
	camera(1,2)=cy;
	camera(2,0)=0.0f;
	camera(2,1)=0.0f;
	camera(2,2)=1.0f;
}
void PCL_analyser::setProjection(Sophus::Sim3f worldToCam)
{
	Eigen::Matrix4f soph =worldToCam.matrix();
	rotTrans =soph.block<3,4>(0,0);
	projection= camera*rotTrans;
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
/*PointCloud::Ptr PCL_registration::getDepth()
{
	PointCloud::Ptr tmp(new PointCloud);
	depthMutex.lock();
	*tmp=*depth;
	depthMutex.unlock();
	return tmp;
}*/
