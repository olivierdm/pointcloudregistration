/**
@file settings.cpp
*/
#include "pointcloudregistration/settings.h"

/// Minimal distance to camera sensor in liveframe that will be considered.
float	minZ=0.1f;
/// Maximal distance from camera sensor in liveframe that will be considered.
float	maxZ=2.0f;
/// Factor that determines the size of depth image with respect to the visual image.
float scaleDepthImage = 3.0f;
/// The size of the structuring element in x direction
float struct_x=3;
/// The size of the structuring element in y direction
float struct_y=2;
float gauss_sigma=2.0f;
int gauss_size=5;
double maxAngle = 8.0;
/// minimum number of lines to withhold candidate
unsigned int minLines = 5;
/// minimum angle to be a stair
float minStairAngle=0.35f;
/// maximum angle to be a stair
float maxStairAngle=1.3f;
/// if the variance of a point in the keyframe is lower than this value it is withheld
float scaledDepthVarTH = 0.01;
/// the number of neighbours a point needs to be valid
int minNearSupport = 5;
/// distance threshold ransac vanishing points
double TH=160000.0;
/// rico threshold ransac parallel lines
double parTH=0.1;
/// tolerance used to calc maximum number of iterations
float eps=0.0001f;
/// cut-off after this
double lastFrameTime = 1e15;
///skew
double skewTH=0.35;
///volume
double volTH=0.05;
/// analyse performance
bool perfmon=true;
