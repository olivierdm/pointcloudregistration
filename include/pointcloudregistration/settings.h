#pragma once

extern float minZ;
extern float maxZ;
extern float scaleDepthImage;
extern float struct_x;
extern float struct_y;
extern float pointTesselation;
extern float lineTesselation;
extern float gauss_sigma;
extern int gauss_size;
extern bool keepInMemory;
extern bool showKFCameras;
extern bool showKFPointclouds;
extern bool showConstraints;
extern bool showCurrentCamera;
extern bool showCurrentPointcloud;

extern float scaledDepthVarTH;
extern float absDepthVarTH;
extern int minNearSupport;
extern int cutFirstNKf;
extern int sparsifyFactor;
extern bool saveAllVideo;

extern int numRefreshedAlready;

extern double lastFrameTime;
