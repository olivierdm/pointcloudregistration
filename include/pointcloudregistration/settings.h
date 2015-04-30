#pragma once

extern float minZ;
extern float maxZ;
extern float scaleDepthImage;
extern float struct_x;
extern float struct_y;
extern float pointTesselation;
extern float lineTesselation;
extern float gauss_sigma;
extern 	double maxAngle;
extern unsigned int minLines; //minimum number of lines to withhold candidate
extern float minStairAngle;//minimum angle to be a stair
extern float maxStairAngle;//maximum angle to be a stair
extern int gauss_size;

extern float scaledDepthVarTH;
extern int minNearSupport;
extern double lastFrameTime;
