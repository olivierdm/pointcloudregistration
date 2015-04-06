#ifndef DATASTRUCTURES_H_
#define DATASTRUCTURES_H_
typedef pcl::PointXYZ Point;
typedef pcl::PointCloud<Point> PointCloud;

class KeyFrame;
/** \brief Struct used for casting the incoming point cloud data. */
struct InputPointDense
{

	float idepth;///< stores the inverse depth
	float idepth_var;///< stores the variance of the inverse depth
	unsigned char color[4];///< array for casting color in RGBA
};
struct GraphConstraint
{
	int from;
	int to;
	float err;
};


struct GraphConstraintPt
{
	KeyFrame* from;
	KeyFrame* to;
	float err;
};
/** \brief Contains the pose of keyframe identified by id*/
struct GraphFramePose
{
	int id;///<keyframe identifier
	float camToWorld[7];//<frame pose
};
#endif /* DATASTRUCTURES_H_ */
