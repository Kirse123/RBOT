#include <QApplication>
#include <QThread>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <filesystem>
#include <stdio.h>
#include <io.h>
#include <string>

#include "object3d.h"
#include "pose_estimator6d.h"
#include "unity_interface.h"

using namespace std;
using namespace cv;

//global vars
int width, height;
float zNear, zFar;
bool isOn = false;
vector<float> distances;
Matx33f K;
Matx14f distCoeffs;
vector<Object3D*> objects;
Mat currentFrame;
Mat* testMat;
PoseEstimator6D* poseEstimator = nullptr;
string models_folder = "";

char *argv[] = { "RBOT_DLL", "arg1", "arg2", NULL };
int argc = sizeof(argv) / sizeof(char*) - 1;
QApplication a(argc, argv);


///<summary>Initiate some RBOT vars and starts QApplication</summary>
int Init(int camera_width, int camera_hight, const char* path) {

	//camera image size (image)
	width = camera_width;
	height = camera_hight;
	
	string str(path);

	// set folder with models
	models_folder = str;
	/*
	for (int i = 0; i < pathLength; ++i) {
		models_folder += path[i];
	}
	*/

	// near and far plane of the OpenGL view frustum
	zNear = 10.0;
	zFar = 10000.0;

	// camera instrinsics
	//K = Matx33f((float)camera_width, 0, (float)camera_width / 2.0, 0, (float)camera_hight, (float)camera_width / 2.0, 0, 0, 1);
	K = Matx33f(650.048, 0, 324.328, 0, 647.183, 257.323, 0, 0, 1);
	distCoeffs = Matx14f(0.0, 0.0, 0.0, 0.0);

	// distances for the pose detection template generation
	distances = { 200.0f, 400.0f, 600.0f };

	//TODO: change initial model's pose
	//Debug==================================================
	string fileName = "E:\\data\\squirrel_demo_low.obj";
	objects.push_back(new Object3D(fileName, 15, -35, 515, 55, -20, 205, 1.0, 0.55f, distances));
	//cout << "size " <<objects.size() << endl;
	//==========================================================

	// create the pose estimator
	poseEstimator = new PoseEstimator6D(width, height, zNear, zFar, K, distCoeffs, objects);

	// active the OpenGL context for the offscreen rendering engine during pose estimation
	RenderingEngine::Instance()->makeCurrent();

	return 0;
}

///<summary>Add 3d-object to vector and loads it from disk</summary>
int AddObj(char* fileName) {
	string fileNameStr(fileName);
	string fullFileName = models_folder + "\\" + fileNameStr;
	//wstring fullFileName = L"E:\\dataÊÐÄ\\squirrel_demo_low.obj";
	//string fullFileName = "E:\\General\\Äîêóìåíòû\\GitHub\\Unity\\Unity_RBOT\\Unity_RBOT\\Assets\\RBOT_Models\\squirrel_demo_low.obj";
	//check if file exists
	if (!FileExists(&fullFileName[0])) {
		return -2;
	}
	//load 3D-object
	objects.push_back(new Object3D(fullFileName, 15, -35, 515, 55, -20, 205, 1.0, 0.55f, distances));

	return 0;
}
///<summary>Updates models 6DOF, according to image</summary>
int GetPose(float* outData) {
	//=============================
	//TextureToCVMat();
	//=============================
	if (currentFrame.empty())
		return -1;
	cout << "size " << objects.size() << endl;
	if (objects.size() == 0) {
		return -2;
	}

	imshow("test", currentFrame);
	waitKey(1);
	// the main pose update call
	poseEstimator->estimatePoses(currentFrame, false, !isOn);

	cout << "Estimated pose " << objects[0]->getPose() << endl;
	

	if (!isOn)
	{
		poseEstimator->toggleTracking(currentFrame, 0, false);
		isOn = true;
		poseEstimator->estimatePoses(currentFrame, false, !isOn);
	}

	Matx44f res = objects[0]->getPose();
	for (int i = 0; i < 4; ++i) {
		for (int j = 0; j < 4; ++j) {
			outData[i * 4 + j] = res(i, j);
		}
	}
	
	return 0;
}
///<summary>Deletes all 3d-objects from memory, destroys pose estimator and stops rendering engine</summary>
void Close() {	
	// deactivate the offscreen rendering OpenGL context
	RenderingEngine::Instance()->doneCurrent();
	// clean up
	RenderingEngine::Instance()->destroy();		//crashes

	for (int i = 0; i < objects.size(); i++)
	{
		delete objects[i];
	}
	objects.clear();
	
	if (poseEstimator != nullptr) {
		delete poseEstimator;		//crashes 
		poseEstimator = nullptr;
	}
}
inline bool fileExists(const string name) {
	struct stat buffer;
	return (stat(name.c_str(), &buffer) == 0);
}
bool FileExists(const char *fname)
{
	return experimental::filesystem::exists(experimental::filesystem::path(fname));
}

int TextureToCVMat() {
	currentFrame = imread("data/frame.png");
	//No frame error
	if (currentFrame.empty())
		return -1;
}
int TextureToCVMat(unsigned char* framePtr, int height, int width) {
	Mat frame(height, width, CV_8UC4, framePtr);
	flip(frame, currentFrame, 0);
	cvtColor(currentFrame, currentFrame, COLOR_BGRA2RGBA);
	//No frame error
	if (currentFrame.empty())
		return -3;
	return 0;
}

int Test() {
	char *argv[] = { "RBOT_DLL", "arg1", "arg2", NULL };
	int argc = sizeof(argv) / sizeof(char*) - 1;
	
	QApplication a(argc, argv);

	cout << "Hello world!" << endl;

	// camera image size
	int width = 640;
	int height = 512;

	// near and far plane of the OpenGL view frustum
	float zNear = 10.0;
	float zFar = 10000.0;

	// camera instrinsics
	//Matx33f K = Matx33f(650.048, 0, 324.328, 0, 647.183, 257.323, 0, 0, 1);
	Matx33f K = Matx33f((float)width, 0, (float)width / 2, 0, (float)height, (float)height / 2, 0, 0, 1);
	Matx14f distCoeffs = Matx14f(0.0, 0.0, 0.0, 0.0);

	// distances for the pose detection template generation
	vector<float> distances = { 200.0f, 400.0f, 600.0f };

	// load 3D objects
	vector<Object3D*> objects;

	//!!!
	//Initial 6DOF pose 
	objects.push_back(new Object3D("data/squirrel_demo_low.obj", 15, -35, 515, 55, -20, 205, 1.0, 0.55f, distances));
	//objects.push_back(new Object3D("data/squirrel_demo_low.obj", 15, -35, 515, 248, -78, 31, 1.0, 0.55f, distances));
	//objects.push_back(new Object3D("data/a_second_model.obj", -50, 0, 600, 30, 0, 180, 1.0, 0.55f, distances2));

	// create the pose estimator
	PoseEstimator6D* poseEstimator = new PoseEstimator6D(width, height, zNear, zFar, K, distCoeffs, objects);

	// move the OpenGL context for offscreen rendering to the current thread, if run in a seperate QT worker thread (unnessary in this example)
	//RenderingEngine::Instance()->getContext()->moveToThread(this);

	// active the OpenGL context for the offscreen rendering engine during pose estimation
	RenderingEngine::Instance()->makeCurrent();

	int timeout = 0;

	bool showHelp = true;

	Mat frame;
	while (true)
	{
		TextureToCVMat();

		// obtain an input image
		frame = currentFrame;

		// the main pose update call
		poseEstimator->estimatePoses(frame, false, true);

		cout << objects[0]->getPose() << endl;

		imshow("result", frame);

		int key = waitKey(timeout);

		// start/stop tracking the first object
		if (key == (int)'1')
		{
			poseEstimator->toggleTracking(frame, 0, false);
			poseEstimator->estimatePoses(frame, false, false);
			timeout = 1;
			showHelp = !showHelp;
		}
		if (key == (int)'2') // the same for a second object
		{
			//poseEstimator->toggleTracking(frame, 1, false);
			//poseEstimator->estimatePoses(frame, false, false);
		}
		// reset the system to the initial state
		if (key == (int)'r')
			poseEstimator->reset();
		// stop the demo
		if (key == (int)'c')
			break;
	}
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
}