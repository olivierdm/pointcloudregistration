#include "pointcloudregistration/KeyFrame.h"
#include "pointcloudregistration/settings.h"
#include <pcl/common/transforms.h>

KeyFrame::KeyFrame(): cloud (new PointCloud), cloudLocal(new PointCloud)
{
	originalInput = 0;
	my_scaledTH = my_absTH =0;
	width=height=0;
	cloudValid=false;
	camToWorld = Sophus::Sim3f();
}
KeyFrame::~KeyFrame()
{
    delete[] originalInput;
}

void KeyFrame::setFrom(lsd_slam_viewer::keyframeMsgConstPtr msg)
{

	// copy over campose.
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
    PointCloud::Ptr pcl_cloud (new PointCloud);
    pcl_cloud->height=1;
    pcl_cloud->width=0;

    ROS_INFO_STREAM("height:"<<height<<"width:"<<width<<"size:"<<msg->pointcloud.size());
   
}

void KeyFrame::refreshPCL()
{
	cloudValid=false;
	bool createLocal=false;
	if(cloudLocal->empty())
	{
		createLocal=true;
	}
		
	cloud->clear();
	my_scaledTH =scaledDepthVarTH;
	my_absTH = absDepthVarTH;
	my_scale = camToWorld.scale();
	my_minNearSupport = minNearSupport;
	my_sparsifyFactor = sparsifyFactor;

	 int err1=0;
    int err2=0;
    int err3=0;
    int num=0;
	float Mdepth=0;
    float my_scale = camToWorld.scale();
    int idx;
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
		if(originalInput[x+y*width].idepth_var * depth4 * my_scale*my_scale > my_absTH){
			err2++;
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
					err3++;
					continue;
				}
			}


		Mdepth+=depth;
           pcl::PointXYZRGB point(originalInput[x+y*width].color[0], originalInput[x+y*width].color[1],originalInput[x+y*width].color[2]);
            point.x=(x*fxi + cxi)*depth;
            point.y=(y*fyi + cyi)*depth;
            point.z=depth;
		cloudLocal->push_back(point);


        }
	Eigen::Matrix4f transformation = camToWorld.matrix();
	pcl::transformPointCloud(*cloudLocal, *cloud, transformation);


	Mdepth/=cloud->width;
       ROS_INFO_STREAM("error 1: "<< err1 << " error 2: "<< err2 << " error 3: " << err3 << " number of points: " << cloud->width << " Mean depth: "<< Mdepth);
cloudValid=true;
}
sensor_msgs::PointCloud2::Ptr KeyFrame::getROSMsg(bool local=true)
{
	if(!cloudValid)
	{
		refreshPCL();
	}
	sensor_msgs::PointCloud2::Ptr pclMsg(new sensor_msgs::PointCloud2) ;
	if(local)
	{
		pcl::toROSMsg(*cloudLocal, *pclMsg);
		pclMsg->header.frame_id = "ardrone_base_frontcam";
	}else{
		pcl::toROSMsg(*cloud, *pclMsg);
		pclMsg->header.frame_id = "world";
	}
	pclMsg->is_dense = true;
	pclMsg->header.seq=id;
	pclMsg->header.stamp=ros::Time(time);
	return pclMsg;
}
PointCloud::Ptr KeyFrame::getPCL()
{
	if(!cloudValid)
	{
		refreshPCL();
	}
return cloud;
}
