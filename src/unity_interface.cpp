#include <QApplication>
#include <QThread>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <filesystem>
#include <stdio.h>
#include <io.h>
#include <string>
#include <vector>

#include "dll_controller.h"
#include "unity_interface.h"

using namespace cv;
using namespace std;

	int CreatePoseEstimator(int camera_width, int camera_height, float inZNear, float inZFar, float* inK, float* inDistCoeffs, float* inDistances) 
	{
		int resCode = DllController::Instance()->CreatePoseEstimator(camera_width, camera_height, inZNear, inZFar, inK, inDistCoeffs, inDistances);

		return resCode;
	}

	int AddObj3d(char* fullFileName, float tx, float ty, float tz, float alpha, float beta, float gamma, float scale, float qualityThreshold, float* templateDistances) {
		
		vector<float> dist = { templateDistances[0], templateDistances[1], templateDistances[2] };

		string tmpFileName = string(fullFileName);
		int resCode = DllController::Instance()->AddObj(tmpFileName, tx, ty, tz, alpha, beta, gamma, scale, qualityThreshold);

		return resCode;
	}

	int RemoveObj3d(int index) 
	{
		int resCode = DllController::Instance()->RemoveObj(index);

		return resCode;
	}

	int ToggleTracking(int objectIndex, bool undistortFrame) {		

		int resCode = DllController::Instance()->ToggleTracking(objectIndex, undistortFrame);

		return resCode;
	}

	int EstimatePose(float* outPoseData, bool undistortFrame, bool checkForLoss) 
	{	
		int resCode = DllController::Instance()->EstimatePose(outPoseData, undistortFrame, checkForLoss);

		return resCode;
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
		int resCode = DllController::Instance()->Reset();

		return resCode;
	}

	bool FileExists(const char *fname)
	{
		return experimental::filesystem::exists(experimental::filesystem::path(fname));
	}