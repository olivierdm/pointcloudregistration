/**
@file settings.cpp
*/

#include "pointcloudregistration/settings.h"


/// Minimal distance to camera sensor in liveframe that will be considered.
float	minZ=0.1f;
/// Maximal distance from camera sensor in liveframe that will be considered.
float	maxZ=4.0f;
/// Factor that determines the size of depth image with respect to the visual image.
float scaleDepthImage = 3.0f;
// new:
float pointTesselation = 1;
float lineTesselation = 2;
float struct_x=3;
float struct_y=2;
float gauss_sigma=2.0f;
int gauss_size=5;
double maxAngle = 8.0;
bool keepInMemory=true;
bool showKFCameras = true;
bool showKFPointclouds = true;
bool showConstraints = true;
bool showCurrentCamera = true;
bool showCurrentPointcloud = true;

float scaledDepthVarTH = 0.01;
int minNearSupport = 5;
int cutFirstNKf = 5;
int sparsifyFactor = 1;

bool saveAllVideo = false;

int numRefreshedAlready = 0;

// cut-off after this
double lastFrameTime = 1e15;
