/**
 *   #, #,         CCCCCC  VV    VV MM      MM RRRRRRR
 *  %  %(  #%%#   CC    CC VV    VV MMM    MMM RR    RR
 *  %    %## #    CC        V    V  MM M  M MM RR    RR
 *   ,%      %    CC        VV  VV  MM  MM  MM RRRRRR
 *   (%      %,   CC    CC   VVVV   MM      MM RR   RR
 *     #%    %*    CCCCCC     VV    MM      MM RR    RR
 *    .%    %/
 *       (%.      Computer Vision & Mixed Reality Group
 *                For more information see <http://cvmr.info>
 *
 * This file is part of RBOT.
 *
 *  @copyright:   RheinMain University of Applied Sciences
 *                Wiesbaden Rüsselsheim
 *                Germany
 *     @author:   Henning Tjaden
 *                <henning dot tjaden at gmail dot com>
 *    @version:   1.0
 *       @date:   30.08.2018
 *
 * RBOT is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * RBOT is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with RBOT. If not, see <http://www.gnu.org/licenses/>.
 */

#include <QApplication>
#include <QThread>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>

#include "object3d.h"
#include "pose_estimator6d.h"

using namespace std;
using namespace cv;

cv::Mat drawResultOverlay(const vector<Object3D*>& objects, const cv::Mat& frame)
{
	// render the models with phong shading
	RenderingEngine::Instance()->setLevel(0);

	vector<Point3f> colors;
	colors.push_back(Point3f(1.0, 0.5, 0.0));
	//colors.push_back(Point3f(0.2, 0.3, 1.0));
	RenderingEngine::Instance()->renderShaded(vector<Model*>(objects.begin(), objects.end()), GL_FILL, colors, true);

	// download the rendering to the CPU
	Mat rendering = RenderingEngine::Instance()->downloadFrame(RenderingEngine::RGB);

	// download the depth buffer to the CPU
	Mat depth = RenderingEngine::Instance()->downloadFrame(RenderingEngine::DEPTH);

	// compose the rendering with the current camera image for demo purposes (can be done more efficiently directly in OpenGL)
	Mat result = frame.clone();
	for (int y = 0; y < frame.rows; y++)
	{
		for (int x = 0; x < frame.cols; x++)
		{
			Vec3b color = rendering.at<Vec3b>(y, x);
			if (depth.at<float>(y, x) != 0.0f)
			{
				result.at<Vec3b>(y, x)[0] = color[2];
				result.at<Vec3b>(y, x)[1] = color[1];
				result.at<Vec3b>(y, x)[2] = color[0];
			}
		}
	}
	return result;
}

int main(int argc, char *argv[])
{
	QApplication a(argc, argv);

	// camera image size
	int width = 640;
	int height = 512;

	// near and far plane of the OpenGL view frustum
	float zNear = 10.0;
	float zFar = 10000.0;

	cout << "Hello! This is my code!" << endl;

	string path = "data";

	// camera instrinsics
	Matx33f K = Matx33f(650.048, 0, 324.328, 0, 647.183, 257.323, 0, 0, 1);
	Matx14f distCoeffs = Matx14f(0.0, 0.0, 0.0, 0.0);

	// distances for the pose detection template generation
	vector<float> distances = { 200.0f, 400.0f, 600.0f };

	// load 3D objects
	vector<Object3D*> objects;
	cout << "loading model" << endl;
	objects.push_back(new Object3D(path + "/squirrel_demo_low.obj", 15, -35, 515, 55, -20, 205, 1.0, 0.55f, distances));
	cout << "model loaded" << endl;

	// create the pose estimator
	PoseEstimator6D* poseEstimator = new PoseEstimator6D(width, height, zNear, zFar, K, distCoeffs, objects);
	cout << "poseEstimator created" << endl;

	// active the OpenGL context for the offscreen rendering engine during pose estimation
	//RenderingEngine::Instance()->makeCurrent();
	cout << "Rendering activated" << endl;

	int timeout = 0;

	bool showHelp = true;

	Mat frame;
	// obtain an input image
	frame = imread(path + "/frame.png");

	// the main pose update call
	poseEstimator->estimatePoses(frame, false, true);
	cout << "pose estimated" << endl;

	stringstream buffer;
	buffer << objects[0]->getPose() << endl;

	string s = buffer.str();

	cout << s;
	cin >> s;

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
}