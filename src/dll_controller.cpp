
#include "dll_controller.h"

	DllController::DllController() 
	{
		poseEstimator = NULL;
	}

	DllController* DllController::controllerInstance = NULL;
	
	char *argv[4] = { "RBOT_DLL", "arg1", "arg2", NULL };
	int argc = sizeof(argv) / sizeof(char*) - 1;
	extern QApplication a{ argc, argv };
	


	//char* DllController::argv[4] = { "RBOT_DLL", "arg1", "arg2", NULL };
	//int DllController::argc = sizeof(argv) / sizeof(char*) - 1;
	//QApplication DllController::a{ argc, argv };

	DllController* DllController::Instance(void) {
		if (controllerInstance == NULL)
			controllerInstance = new DllController();
		return controllerInstance;	
	}

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

	int DllController::Init(std::string path, int camera_width, int camera_hight, float inZNear, float inZFar, float* inK, float* inDistCoeffs)
	{		
		//camera image size (image)
		width = camera_width;
		height = camera_hight;

		// set folder with models
		models_folder = path;

		// near and far plane of the OpenGL view frustum
		zNear = inZNear;
		zFar = inZNear;

		// camera instrinsics
		//K = Matx33f((float)camera_width, 0, (float)camera_width / 2.0, 0, (float)camera_hight, (float)camera_width / 2.0, 0, 0, 1);
		K = cv::Matx33f(inK[0], inK[1], inK[2], inK[3], inK[4], inK[5], inK[6], inK[7], inK[8]);
		distCoeffs = cv::Matx14f(inDistCoeffs[0], inDistCoeffs[1], inDistCoeffs[2], inDistCoeffs[3]);

		// distances for the pose detection template generation
		distances = { 200.0f, 400.0f, 600.0f };

		//TODO: change initial model's pose
		//Debug==================================================
		//std::string fileName = "E:\\data\\squirrel_demo_low.obj";
		//objects.push_back(new Object3D(fileName, 15, -35, 515, 55, -20, 205, 1.0, 0.55f, distances));
		//cout << "size " <<objects.size() << endl;
		//==========================================================

		// create the pose estimator
		//poseEstimator = new PoseEstimator6D(width, height, zNear, zFar, K, distCoeffs, objects);

		// active the OpenGL context for the offscreen rendering engine during pose estimation
		RenderingEngine::Instance()->makeCurrent();

		initialized = true;

		return 0;
	}

	int DllController::EstimatePose(float* outData, bool undistortFrame, bool checkForLoss) 
	{
		//===========Debug frame=========================
		currentFrame = cv::imread("E:\\data\\frame.png");
		//===============================================
		if (currentFrame.empty())
			return -1;
		//std::cout << "size " << objects.size() << endl;
		if (objects.size() == 0) {
			return -2;
		}

		//=================Debug output==========================
		imshow("test", currentFrame);
		cv::waitKey(1);
		//=======================================================

		// the main pose update call
		poseEstimator->estimatePoses(currentFrame, undistortFrame, checkForLoss);

		// debug console output
		//std::cout << "Estimated pose " << objects[0]->getPose() << endl;

		cv::Matx44f res = objects[0]->getPose();
		for (int i = 0; i < 4; ++i) {
			for (int j = 0; j < 4; ++j) {
				outData[i * 4 + j] = res(i, j);
			}
		}

		return 0;
	}

	int DllController::AddObj(std::string fileName, float tx, float ty, float tz, float alpha, float beta, float gamma, float scale, float qualityThreshold, std::vector<float> &templateDistances) 
	{		
		std::string fullFileName = models_folder + "\\" + fileName;
		//wstring fullFileName = L"E:\\dataÊÐÄ\\squirrel_demo_low.obj";
		//string fullFileName = "E:\\General\\Äîêóìåíòû\\GitHub\\Unity\\Unity_RBOT\\Unity_RBOT\\Assets\\RBOT_Models\\squirrel_demo_low.obj";
		//check if file exists
		if (!FileExists(&fullFileName[0])) {
			return -2;
		}
		//load 3D-object
		objects.push_back(new Object3D(fullFileName, tx, ty, tz, alpha, beta, gamma, scale, qualityThreshold, templateDistances));

		if (poseEstimator != NULL)
			delete poseEstimator;
		poseEstimator = new PoseEstimator6D(width, height, zNear, zFar, K, distCoeffs, objects);
		
		return 0;
	}

	int DllController::ToggleTracking(int objectIndex, bool undistortFrame) 
	{
		// Object not found
		if (objectIndex > objects.size() - 1)
			return -2;

		poseEstimator->toggleTracking(currentFrame, objectIndex, undistortFrame);
	}

	int DllController::TextureToCVMat(unsigned char* framePtr, int frame_height, int frame_width) 
	{		
		this->width = frame_width;
		this->height = frame_height;

		cv::Mat frame(height, width, CV_8UC4, framePtr);
		cv::flip(frame, currentFrame, 0);
		cv::cvtColor(currentFrame, currentFrame, cv::COLOR_BGRA2RGBA);
		
		// no frame error
		if (currentFrame.empty())
			return -3;
		return 0;
	}

	int DllController::Reset() 
	{
		poseEstimator->reset();

		return 0;
	}

	bool DllController::IsInitialized() {
		return initialized;
	}

	bool DllController::FileExists(const char *fname)
	{
		return std::experimental::filesystem::exists(std::experimental::filesystem::path(fname));
	}