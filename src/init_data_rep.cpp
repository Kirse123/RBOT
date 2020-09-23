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

using namespace std;
using namespace cv;

class InitDataRepo {
public:


	InitDataRepo() {

	}
};