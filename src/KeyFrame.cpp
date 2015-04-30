//! KeyFrame class
/*! 
This class is used to store the received keyframes and their attributes.
*/
#include <ros/ros.h>
#include "pointcloudregistration/KeyFrame.h"
#include "pointcloudregistration/settings.h"
#include <pcl/common/transforms.h>
#include <pcl/filters/statistical_outlier_removal.h>

KeyFrame::KeyFrame(): width(0), height(0), my_scaledTH(0.0f), cloud (new PointCloud), originalInput(0)
{
/// 
/// \brief Default constructor, solely used to initialize private members.
///
	camToWorld = Sophus::Sim3f();
}
KeyFrame::~KeyFrame()
{
///
/// \brief Default destructor that clears the inputdata.
///
	delete[] originalInput;
}

void KeyFrame::setFrom(lsd_slam_viewer::keyframeMsgConstPtr msg)
{
///
/// \brief Copies the data from the incoming message.
///
/// Copies the camera pose and camera parameters. Then it checks if the input is correctly sized and casts the pointcloud data.
/// @param[in] msg keyframe message
/// 
	memcpy(camToWorld.data(), msg->camToWorld.data(), 7*sizeof(float));

	fx = msg->fx;
	fy = msg->fy;
	cx = msg->cx;
	cy = msg->cy;

	fxi = 1/fx;
	fyi = 1/fy;
	cxi = -cx / fx;
	cyi = -cy / fy;

	width = msg->width;
	height = msg->height;
	id = msg->id;
	time = msg->time;

    if(originalInput != 0)
        delete[] originalInput;
    originalInput=0;


    if(msg->pointcloud.size() != width*height*sizeof(InputPointDense))
    {
	if(msg->pointcloud.size() != 0)
        {
            ROS_WARN_STREAM("PC with points, but number of points not right! is "<< msg->pointcloud.size()
                            << "should be" << sizeof(InputPointDense) << "x" << width << "x" << height << "="
                            << width*height*sizeof(InputPointDense));
		return;
        }
	else
	{	if(msg->isKeyframe)
			ROS_WARN("size not correct of a keyframe");
		return;
	}
    }
    else
    {
        originalInput = new InputPointDense[width*height];
        memcpy(originalInput, msg->pointcloud.data(), width*height*sizeof(InputPointDense));
    }
    ROS_INFO_STREAM("height:"<<height<<"width:"<<width<<"size:"<<msg->pointcloud.size());
   
}

void KeyFrame::refreshPCL()
{
///
/// \brief Renders or rerenders the pcl cloud from the incoming data.
///
/// Checks if the local parameters still correspond to the global parameters. If not the pcl cloud is cleared and the input data is rendered in the pcl cloud.
///
	bool paramsStillGood = my_scaledTH == scaledDepthVarTH &&
		my_scale*1.2 > camToWorld.scale() &&
		my_scale < camToWorld.scale()*1.2 &&
		my_minNearSupport == minNearSupport;
	if(paramsStillGood){
	//ROS_INFO("params still good");
		return;
		}
	cloud.reset(new PointCloud);
	my_scaledTH =scaledDepthVarTH;
	my_scale = camToWorld.scale();
	my_minNearSupport = minNearSupport;

	 int err1=0;
    int err2=0;
	float Mdepth=0;
	PointCloud::Ptr unfiltered(new PointCloud);
    for(int y=1; y<height-1; y++)
        for(int x=1; x<width-1; x++)
        {
            float depth = 1.0 / originalInput[x+y*width].idepth;
            float depth4 = depth*depth;
            depth4*= depth4;
            if(originalInput[x+y*width].idepth_var * depth4 > my_scaledTH){
		err1++;
		continue;
		}
		if(my_minNearSupport > 1)
		{
			int nearSupport = 0;
			for(int dx=-1;dx<2;dx++)
				for(int dy=-1;dy<2;dy++)
				{
					int idx = x+dx+(y+dy)*width;
					if(originalInput[idx].idepth > 0)
					{
						float diff = originalInput[idx].idepth - 1.0f / depth;
						if(diff*diff < 2*originalInput[x+y*width].idepth_var)
							nearSupport++;
					}
				}

			if(nearSupport < my_minNearSupport){
				err2++;
				continue;
			}
		}

		Mdepth+=depth;
         //  pcl::PointXYZRGB point(originalInput[x+y*width].color[0], originalInput[x+y*width].color[1],originalInput[x+y*width].color[2]);
	Point point;
            point.x=(x*fxi + cxi)*depth;
            point.y=(y*fyi + cyi)*depth;
            point.z=depth;
		cloud->push_back(point);
        }
  /*pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
  sor.setInputCloud (unfiltered);
  sor.setMeanK (10);
  sor.setStddevMulThresh (2.0);
  sor.filter (*cloud);*/
	Mdepth/=cloud->width;
       ROS_INFO_STREAM("cloud "<< id << ": error 1: "<< err1 << " error 2: "<< err2 << " number of points: " << cloud->width << " Mean depth: "<< Mdepth);
}
/*sensor_msgs::PointCloud2::Ptr KeyFrame::getROSMsg()
{
///
/// \brief converts the local pointcloud to a ros compatible message and passes the pointer.
///
	boost::mutex::scoped_lock lock(cloudMutex);
	sensor_msgs::PointCloud2::Ptr pclMsg(new sensor_msgs::PointCloud2) ;
	refreshPCL();
	pcl::toROSMsg(*cloud, *pclMsg);
	pclMsg->header.frame_id = "ardrone_base_frontcam";
	pclMsg->is_dense = true;
	pclMsg->header.seq=id;
	pclMsg->header.stamp=ros::Time(time);
	return pclMsg;
}*/
PointCloud::Ptr KeyFrame::getPCL()
{
///
/// \brief Passes back a pointer to the local cloud and locks the cloud for internal use.
///
/// The cloudMutex is locked and the pointer is passed. This method has a blocking behauviour if lock is optained by other local method. KeyFrame::release() can be used to unlock the lock.
///
	//cloudMutex.lock();
	refreshPCL();
	ROS_DEBUG_STREAM("locking cloud "<< id << "size: " <<cloud->width);
	return cloud;
}
/*void KeyFrame::release()
{
///
/// \brief Releases the lock. Schould be used after KeyFrame::getPcl().
///
	ROS_DEBUG_STREAM("unlocking cloud "<< id );
	cloudMutex.unlock();
}*/
