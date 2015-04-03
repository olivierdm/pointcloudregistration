#pragma once

extern float minZ;
extern float maxZ;
extern float scaleDepthImage;
extern float pointTesselation;
extern float lineTesselation;

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
