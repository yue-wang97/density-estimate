#pragma once
#include "opencv2/imgproc.hpp"
#include <opencv2\opencv.hpp>

using namespace std;
using namespace cv;

#include "MyCanny.h"

#define BGMOG_DEFAULT_ALPHA			0.01
#define BGMOG_DEFAULT_THRESHOLD		0.7

#define BGMOG_DEFAULT_FREQ0			1
#define BGMOG_DEFAULT_FREQ1			1

#define BGMOG_FOREGROUND_MASK		1

class edgeBackgroundMOG: public MyCanny
{
private:
	unsigned d_alpha;
	ushort thresholdU;
	uchar freq[2];

	Mat backgroundFreq[2];
	Mat background[CANNY_ORIENTATION_NUM];

	void releaseData();
	bool sortBKWeight(uchar index0, ushort* w);
	void initBackground(int rows, int cols);

public:
	edgeBackgroundMOG();
	edgeBackgroundMOG(float alpha, float threshold = BGMOG_DEFAULT_THRESHOLD, int freq0 = BGMOG_DEFAULT_FREQ0, int freq1 = BGMOG_DEFAULT_FREQ1);
	~edgeBackgroundMOG();

	void getForeground(const Mat& cannyEdges, Mat& fgMask);
	void updateBackground(const Mat& cannyEdges);
};

