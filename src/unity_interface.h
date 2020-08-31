#ifndef UNITY_INTERFACE_H
#define UNITY_INTERFACE_H

#ifdef RBOT_EXPORTS
#define RBOT_API __declspec(dllexport)
#else
#define RBOT_API __declspec(dllimport)
#endif // RBOT_EXPORTS

extern "C" RBOT_API int GetPose(float* outData);

#endif // ! UNITY_INTERFACE_H

