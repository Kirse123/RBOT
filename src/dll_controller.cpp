#include "dll_controller.h"	

using namespace cv;
using namespace std;

	DllController::DllController() 
	{
		poseEstimator = NULL;
	}

	DllController* DllController::controllerInstance = NULL;

	DllController::~DllController()
	{
		// deactivate the offscreen rendering OpenGL context
		RenderingEngine::Instance()->doneCurrent();
		// clean up
		RenderingEngine::Instance()->destroy();		

		for (int i = 0; i < objects.size(); i++)
		{
			delete objects[i];
		}
		objects.clear();

		if (poseEstimator != nullptr) {
			delete poseEstimator;		
			poseEstimator = nullptr;
		}
	}

	int DllController::CreatePoseEstimator(int camera_width, int camera_height, float inZNear, float inZFar, float* inK, float* inDistCoeffs, float* inDistances)
	{		
		if (qApp == NULL || qApp == nullptr) {
			char *argv[] = { "RBOT_DLL", "arg1", "arg2", NULL };
			int argc = sizeof(argv) / sizeof(char*) - 1;
			QApplication* a = new QApplication(argc, argv);
		}
		
		if (qApp == NULL || qApp == nullptr)
		{
			return -6;
		}

		if (DllController::Instance()->objects.size() == 0) {
			return -2;
		}


		//camera image size (image)
		width = camera_width;
		height = camera_height;

		// near and far plane of the OpenGL view frustum
		zNear = inZNear;
		zFar = inZFar;

		// camera instrinsics
		//K = Matx33f((float)camera_width, 0, (float)camera_width / 2.0, 0, (float)camera_hight, (float)camera_width / 2.0, 0, 0, 1);
		K = Matx33f(inK[0], inK[1], inK[2], inK[3], inK[4], inK[5], inK[6], inK[7], inK[8]);
		distCoeffs = Matx14f(inDistCoeffs[0], inDistCoeffs[1], inDistCoeffs[2], inDistCoeffs[3]);

		// distances for the pose detection template generation
		distances = { inDistances[0], inDistances[1], inDistances[2] };

		// Delete old pose estimator
		if (poseEstimator != NULL || poseEstimator != nullptr)
		{
			delete poseEstimator;
		}

		// create new  pose estimator
		poseEstimator = new PoseEstimator6D(width, height, zNear, zFar, K, distCoeffs, objects);

		//RenderingEngine::Instance()->getContext()->moveToThread(a->thread());

		// active the OpenGL context for the offscreen rendering engine during pose estimation
		RenderingEngine::Instance()->makeCurrent();

		return 0;
	}

	int DllController::EstimatePose(float* outData, bool undistortFrame, bool checkForLoss) 
	{
		if (poseEstimator == NULL || poseEstimator == nullptr)
			return -1;
		//===========Debug frame=========================
		//this->currentFrame = imread("E:\\data\\frame.png");
		//===============================================
		if (currentFrame.empty())
			return -3;
		cout << "[In EstimatePose] objetcs.size(): " << objects.size() << endl;
		if (objects.size() == 0) {
			return -2;
		}
		//=================Debug output==========================
		imshow("Current Frame", currentFrame);
		waitKey(1);
		//=======================================================
		
		// the main pose update call
		poseEstimator->estimatePoses(currentFrame, false, true);

		cout << "TRS-MAT: " << objects[0]->getPose() << endl;

		Matx44f res = objects[0]->getPose();
		for (int i = 0; i < 4; ++i) {
			for (int j = 0; j < 4; ++j) {
				outData[i * 4 + j] = res(i, j);
			}
		}

		return 0;
	}

	int DllController::AddObj(string fullFileName, float tx, float ty, float tz, float alpha, float beta, float gamma, float scale, float qualityThreshold, vector<float> &templateDistances) 
	{		
		if (!FileExists(&fullFileName[0])) {
			return -5;
		}
		//load 3D-object
		objects.push_back(new Object3D(fullFileName, tx, ty, tz, alpha, beta, gamma, scale, qualityThreshold, templateDistances));

		return 0;
	}

	int DllController::RemoveObj(int index) 
	{
		if (objects.size() < index + 1)
			// out of range error
			return -2;
		
		this->objects.erase (objects.begin() + index);
		return 0;
	}

	int DllController::ToggleTracking(int objectIndex, bool undistortFrame) 
	{
		if (poseEstimator == NULL || poseEstimator == nullptr)
			return -1;
		// Object not found
		if (objectIndex > objects.size() - 1)
			return -2;

		poseEstimator->toggleTracking(currentFrame, objectIndex, undistortFrame);

		poseEstimator->estimatePoses(currentFrame, false, false);
	}

	int DllController::TextureToCVMat(unsigned char* framePtr, int frame_height, int frame_width) 
	{		
		this->width = frame_width;
		this->height = frame_height;

		Mat frame(height, width, CV_8UC4, framePtr);
		flip(frame, currentFrame, 0);
		cvtColor(currentFrame, currentFrame, cv::COLOR_BGRA2RGBA);
		
		// no frame error
		if (currentFrame.empty())
			return -3;
		return 0;
	}

	int DllController::Reset() 
	{
		if (poseEstimator == NULL || poseEstimator == nullptr)
			return -1;
		poseEstimator->reset();

		return 0;
	}

	bool DllController::IsInitialized() {
		return initialized;
	}

	bool DllController::FileExists(const char *fname)
	{
		return experimental::filesystem::exists(experimental::filesystem::path(fname));
	}