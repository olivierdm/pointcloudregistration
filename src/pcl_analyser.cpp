

PCL_registration::PCL_registration(): cloud (new PointCloud), depth(new PointCloud)
{
	nh=ros::NodeHandle("~");
	pub = nh.advertise<sensor_msgs::PointCloud2> ("points2", 1);
	graph= (new KeyFrameGraph);
	currentCamID=0;
	minX = maxX = minY = maxY=0.0;
	minZ= 0.3;
	maxZ=3;
	inter=10;
	ROS_INFO("registration ready");
     //ctor
}
void PCL_registration::getDepthImage(lsd_slam_viewer::keyframeMsgConstPtr msg)
{

		if(currentCamID > msg->id)
		{
			printf("detected backward-jump in id (%d to %d), resetting!\n", currentCamID, msg->id);
		}

//		lastAnimTime = lastCamTime = msg->time;
		currentCamID = msg->id;
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

    }
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
	float minx,minX,maxx,maxX,miny,minY,maxy,maxY;
	float width = (float) msg->width;
	float height = (float) msg->height;
	minX=(fxi+cxi)*maxZ;
	maxX=(width*fxi + cxi)*maxZ;

	minx=(fxi+cxi)*minZ;
	maxx=(width*fxi + cxi)*minZ;

	minY=(fyi+cyi)*maxZ;
	maxY=(height*fyi + cyi)*maxZ;

	miny=(fyi+cyi)*maxZ;
	maxY=(height*fyi + cyi)*maxZ;
	
	hullcloud.pushback()
}
PointCloud::Ptr PCL_registration::getDepth()
{
	PointCloud::Ptr tmp(new PointCloud);
	depthMutex.lock();
	*tmp=*depth;
	depthMutex.unlock();
	return tmp;
}
