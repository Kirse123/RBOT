#include <QApplication>
#include <QThread>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <filesystem>
#include <stdio.h>
#include <io.h>
#include <string>
#include <vector>

#include "object3d.h"
#include "pose_estimator6d.h"
#include "unity_interface.h"
#include "dll_controller.h"

using namespace cv;

	char *argv[4] = { "RBOT_DLL", "arg1", "arg2", NULL };
	int argc = sizeof(argv) / sizeof(char*) - 1;
	QApplication a(argc, argv);

	int Init(const char* path, int camera_width, int camera_height, float inZNear, float inZFar, float* inK, float* inDistCoeffs) {

		std::string tmpStr = std::string(path);
		int resCode = DllController::Instance()->Init(tmpStr, camera_width, camera_height, inZNear, inZFar, inK, inDistCoeffs);

		return resCode;
	}

	int AddObj3d(char* fullFileName, float tx, float ty, float tz, float alpha, float beta, float gamma, float scale, float qualityThreshold, float* templateDistances) {
		
		std::vector<float> dist = { templateDistances[0], templateDistances[1], templateDistances[2] };

		std::string tmpFileName = std::string(fullFileName);
		int resCode = DllController::Instance()->AddObj(tmpFileName, tx, ty, tz, alpha, beta, gamma, scale, qualityThreshold);

		return resCode;
	}

	int ToggleTracking(int objectIndex, bool undistortFrame) {
		
		if (!DllController::Instance()->IsInitialized()) {
			return -4;
		}

		int resCode = DllController::Instance()->ToggleTracking(objectIndex, undistortFrame);

		return 0;
	}

	int EstimatePose(float* outData, bool undistortFrame, bool checkForLoss) 
	{
		if (!DllController::Instance()->IsInitialized())
		{
			return -4;
		}
	
		int resCode = DllController::Instance()->EstimatePose(outData, undistortFrame, checkForLoss);

		return resCode;
	}

	void Close() {	

	}

	int TextureToCVMat(unsigned char* framePtr, int height, int width) 
	{
		
		if (!DllController::Instance()->IsInitialized()) {
			return -4;
		}
		
		int resCode = DllController::Instance()->TextureToCVMat(framePtr, height, width);

		return resCode;
	}

	int Reset() 
	{
		if (!DllController::Instance()->IsInitialized()) {
			return -4;
		}

		int resCode = DllController::Instance()->Reset();

		return resCode;
	}

	inline bool fileExists(const std::string name) {
		struct stat buffer;
		return (stat(name.c_str(), &buffer) == 0);
	}
	bool FileExists(const char *fname)
	{
		return std::experimental::filesystem::exists(std::experimental::filesystem::path(fname));
	}
	/*
	int TextureToCVMat() {
		currentFrame = imread("data/frame.png");
		//No frame error
		if (currentFrame.empty())
			return -1;
	}
	*/