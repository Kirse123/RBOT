#include <QApplication>
#include <QThread>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>

#include "object3d.h"
#include "pose_estimator6d.h"

#include "unity_interface.h"

using namespace std;
using namespace cv;

//global vars
int width, height;
float zNear, zFar;
vector<float> distances;
Matx33f K;
Matx14f distCoeffs;
vector<Object3D*> objects;
Mat currentFrame;
PoseEstimator6D* poseEstimator;

///<summary>Initiate some RBOT vars and starts QApplication</summary>
int Init(int camera_width, int camera_hight) {
	char *argv[] = { "RBOT_DLL", "arg1", "arg2", NULL };
	int argc = sizeof(argv) / sizeof(char*) - 1;

	QApplication a(argc, argv);

	// camera image size (image)
	width = 640;
	height = 512;

	// near and far plane of the OpenGL view frustum
	zNear = 10.0;
	zFar = 10000.0;

	// camera instrinsics
	K = Matx33f(650.048, 0, 324.328, 0, 647.183, 257.323, 0, 0, 1);
	distCoeffs = Matx14f(0.0, 0.0, 0.0, 0.0);

	// distances for the pose detection template generation
	distances = { 200.0f, 400.0f, 600.0f };
	return 0;
}

///<summary>Add 3d-object to vector and loads it from disk</summary>
int AddObj(string fullFileName) {
	//check if file exists
	if (!fileExists(fullFileName)) {
		return -1;
	}
	//load 3D-object
	objects.push_back(new Object3D(fullFileName, 15, -35, 515, 55, -20, 205, 1.0, 0.55f, distances));
}

///<summary>Converts raw image data from Unity into CVMat</summary>
int TextureToCVMat(unsigned char* framePtr, int height, int width) {
	Mat frame(height, width, CV_8UC4, framePtr);
	flip(frame, currentFrame, 0);
	//No frame error
	if (currentFrame.empty())
		return -1;
}

///<summary>Converts image on a disk into CVMat</summary>
int TextureToCVMat(string imgFullName) {
	currentFrame = imread("data/frame.png");
	//No frame error
	if (currentFrame.empty())
		return -1;
}

inline bool fileExists(const string name) {
	struct stat buffer;
	return (stat(name.c_str(), &buffer) == 0);
}
std::string GetPose(string path) {

	char *argv[] = { "RBOT_DLL", "arg1", "arg2", NULL };
	int argc = sizeof(argv) / sizeof(char*) - 1;
	
	QApplication a(argc, argv);

	//cout << "Hello! No QApp" << endl;

	// camera image size
	int width = 640;
	int height = 512;

	// near and far plane of the OpenGL view frustum
	float zNear = 10.0;
	float zFar = 10000.0;

	// camera instrinsics
	Matx33f K = Matx33f(650.048, 0, 324.328, 0, 647.183, 257.323, 0, 0, 1);
	Matx14f distCoeffs = Matx14f(0.0, 0.0, 0.0, 0.0);

	// distances for the pose detection template generation
	vector<float> distances = { 200.0f, 400.0f, 600.0f };

	// load 3D objects
	vector<Object3D*> objects;
	//cout << "loading model" << endl;
	objects.push_back(new Object3D(path + "/squirrel_demo_low.obj", 15, -35, 515, 55, -20, 205, 1.0, 0.55f, distances));
	//cout << "model loaded" << endl;

	// create the pose estimator
	PoseEstimator6D* poseEstimator = new PoseEstimator6D(width, height, zNear, zFar, K, distCoeffs, objects);
	//cout << "poseEstimator created" << endl;

	// active the OpenGL context for the offscreen rendering engine during pose estimation
	//RenderingEngine::Instance()->makeCurrent();
	//cout << "Rendering activated" << endl;

	int timeout = 0;

	bool showHelp = true;

	Mat frame;
	// obtain an input image
	frame = imread(path + "/frame.png");

	// the main pose uodate call
	poseEstimator->estimatePoses(frame, false, true);
	//cout << "pose estimated" << endl;

	stringstream buffer;
	buffer << objects[0]->getPose() << endl;

	string s = buffer.str();

	//cout << s;
	//cin >> s;

	// deactivate the offscreen rendering OpenGL context
	//RenderingEngine::Instance()->doneCurrent();

	// clean up
	//RenderingEngine::Instance()->destroy();

	for (int i = 0; i < objects.size(); i++)
	{
		delete objects[i];
	}
	objects.clear();

	delete poseEstimator;

	return s;
}

int GetPose(float* outData) {
	char *argv[] = { "RBOT_DLL", "arg1", "arg2", NULL };
	int argc = sizeof(argv) / sizeof(char*) - 1;

	QApplication a(argc, argv);

	// camera image size (image)
	width = 640;
	height = 512;

	// near and far plane of the OpenGL view frustum
	zNear = 10.0;
	zFar = 10000.0;

	// camera instrinsics
	K = Matx33f(650.048, 0, 324.328, 0, 647.183, 257.323, 0, 0, 1);
	distCoeffs = Matx14f(0.0, 0.0, 0.0, 0.0);

	// distances for the pose detection template generation
	distances = { 200.0f, 400.0f, 600.0f };
	
	//check if file exists
	if (!fileExists("data/squirrel_demo_low.obj")) {
		return -2;
	}

	// load 3D objects
	objects.push_back(new Object3D("data/squirrel_demo_low.obj", 15, -35, 515, 55, -20, 205, 1.0, 0.55f, distances));

	// create the pose estimator
	PoseEstimator6D* poseEstimator = new PoseEstimator6D(width, height, zNear, zFar, K, distCoeffs, objects);

	// active the OpenGL context for the offscreen rendering engine during pose estimation
	RenderingEngine::Instance()->makeCurrent();

	int timeout = 0;

	bool showHelp = true;

	Mat frame;
	// obtain an input image
	frame = imread("data/frame.png");
	//No frame error
	if (frame.empty())
		return -1;

	// the main pose uodate call
	poseEstimator->estimatePoses(frame, false, true);
	//cout << "pose estimated" << endl;

	Matx44f &res = objects[0]->getPose();
	for (int i = 0; i < 4; ++i) {
		for (int j = 0; j < 4; ++j) {
			outData[i * 4 + j] = res(i, j);
		}
	}
	
	//Debug output--------------------------
	//for (int i = 0; i < 16; ++i) {
	//	cout << outData[i] << " ";
	//}
	//cout << endl;
	//--------------------------------------


	// deactivate the offscreen rendering OpenGL context
	RenderingEngine::Instance()->doneCurrent();

	// clean up
	RenderingEngine::Instance()->destroy();

	for (int i = 0; i < objects.size(); i++)
	{
		delete objects[i];
	}
	objects.clear();

	delete poseEstimator;

	return 0;
}

///<summary>Deletes all 3d-objects from memory, destroys pose estimator and stops rendering engine</summary>
int Destroy() {
	// deactivate the offscreen rendering OpenGL context
	RenderingEngine::Instance()->doneCurrent();

	// clean up
	RenderingEngine::Instance()->destroy();

	for (int i = 0; i < objects.size(); i++)
	{
		delete objects[i];
	}
	objects.clear();

	delete poseEstimator;

	return 0;
}