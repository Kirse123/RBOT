#ifndef UNITY_INTERFACE_H
#define UNITY_INTERFACE_H

#define RBOT_API __declspec(dllexport) __stdcall

///<summary>Updates models 6DOF, according to image</summary>
extern "C" int RBOT_API EstimatePose(float* outData, bool undistortFrame, bool checkForLoss);

///<summary>Initiate some RBOT vars and starts QApplication</summary>
extern "C" int RBOT_API Init(const char* path, int camera_width, int camera_hight, float inZNear, float inZFar, float* inK, float* inDistCoeffs);

extern "C" void RBOT_API Close();

///<summary>Converts raw image data from Unity into CVMat</summary>
extern "C" int RBOT_API TextureToCVMat(unsigned char* framePtr, int height, int width);

///<summary>Add 3d-object to vector and loads it from disk</summary>
extern "C" int RBOT_API AddObj3d(char* fileName, float tx, float ty, float tz, float alpha, float beta, float gamma, float scale, float qualityThreshold, float* templateDistances);

///<summary>Toggle tracking for selected object</summary>
extern "C" int RBOT_API ToggleTracking(int objectIndex, bool undistortFrame);

///<summary>Resets/stops pose tracking for all objects by clearing the respective sets of tclc - histograms.</summary>
extern "C" int RBOT_API Reset();

#endif // ! UNITY_INTERFACE_H

