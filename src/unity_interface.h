#ifndef UNITY_INTERFACE_H
#define UNITY_INTERFACE_H

//#ifdef RBOT_EXPORTS
#define RBOT_API __declspec(dllexport) __stdcall
//#else
//#define RBOT_API __declspec(dllimport)
//#endif // RBOT_EXPORTS

extern "C" int RBOT_API GetPose(float* outData);
extern "C" int RBOT_API Init(int camera_width, int camera_hight, const char* path);
extern "C" void RBOT_API Close();
///<summary>Converts raw image data from Unity into CVMat</summary>
extern "C" int RBOT_API TextureToCVMat(unsigned char* framePtr, int height, int width);
extern "C" int RBOT_API AddObj(char* fileName);
extern "C" int RBOT_API Test();

///<summary>Converts image on a disk into CVMat</summary>
int TextureToCVMat();
inline bool fileExists(const std::string name);
bool FileExists(const char *fname);

#endif // ! UNITY_INTERFACE_H

