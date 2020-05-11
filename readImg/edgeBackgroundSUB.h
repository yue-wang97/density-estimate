#pragma once
#include "opencv2/imgproc.hpp"
#include <opencv2\opencv.hpp>

using namespace std;
using namespace cv;

#include "MyCanny.h"

#define BG_SUB_DEFAULT_INTERVAL		1
#define BG_SUB_DEFAULT_CHANGE_A		2000
#define BG_SUB_DEFAULT_CHANGE_R		0.5

#define BG_SUB_FRAME_NUM			3
#define BG_SUB_FOREGROUND_MASK		1

class edgeBackgroundSUB: public MyCanny
{
private:
	int background_interval;
	int foreground_max_change_a; double foreground_max_change_r;

	int fgNum;
	int interval;

	Mat background[BG_SUB_FRAME_NUM];
	uchar* backgroundData[BG_SUB_FRAME_NUM];
	int backgroundLast, background_num;

	void releaseData();
	void initBackground(int rows, int cols);

public:
	edgeBackgroundSUB();
	edgeBackgroundSUB(int interval, int max_change_a = BG_SUB_DEFAULT_CHANGE_A, double max_change_r = BG_SUB_DEFAULT_CHANGE_R);
	~edgeBackgroundSUB();

	void getParameters(int& interval, int& max_change_a, double& max_change_r);
	void setParameters(int interval, int max_change_a = BG_SUB_DEFAULT_CHANGE_A, double max_change_r = BG_SUB_DEFAULT_CHANGE_R);
	void reset();

	bool getForeground(const Mat& cannyEdges, Mat& fgMask);
	void updateBackground(const Mat& cannyEdges);
};

