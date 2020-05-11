#include "stdafx.h"
#include "edgeBackgroundMOG.h"

#define BGMOG_WEIGHT_MAX					64000				//MAX*(MAX+4.5)<UINT32_MAX
#define BGMOG_WEIGHT_USHORT(w)			(    ushort( float(BGMOG_WEIGHT_MAX) * (w) + float(0.5) ) )
#define BGMOG_WEIGHT_UNSIGNED(w)			( unsigned( float(BGMOG_WEIGHT_MAX) * (w) + float(0.5) ) )

edgeBackgroundMOG::edgeBackgroundMOG()
{
	double alpha = BGMOG_DEFAULT_ALPHA;
	d_alpha = BGMOG_WEIGHT_UNSIGNED(alpha / (1.0 - alpha));

	thresholdU = BGMOG_WEIGHT_USHORT(BGMOG_DEFAULT_THRESHOLD);

	freq[0] = BGMOG_DEFAULT_FREQ0;
	freq[1] = BGMOG_DEFAULT_FREQ1;
}

edgeBackgroundMOG::edgeBackgroundMOG(float alpha, float threshold, int freq0, int freq1)
{
	d_alpha = BGMOG_WEIGHT_UNSIGNED(alpha / (1.0 - alpha));

	thresholdU = BGMOG_WEIGHT_USHORT(threshold);

	freq[0] = uchar(freq0);
	freq[1] = uchar(freq1);
}

edgeBackgroundMOG::~edgeBackgroundMOG()
{
	releaseData();
}

void edgeBackgroundMOG::releaseData()
{
	backgroundFreq[0].release();
	backgroundFreq[1].release();

	for (int i = 0; i < CANNY_ORIENTATION_NUM; i++)
		background[i].release();

	return;
}

void edgeBackgroundMOG::initBackground(int rows, int cols)
{
	//release old data
	releaseData();

	//create new data
	if (rows == 0 || cols == 0)
		return;

	backgroundFreq[0].create(rows, cols, CV_8U);
	backgroundFreq[1].create(rows, cols, CV_8U);

	for (int i = 0; i < CANNY_ORIENTATION_NUM; i++)
		background[i].create(rows, cols, CV_16UC1);

	//init freq
	size_t len = rows * cols;
	memset(backgroundFreq[0].data, 0, len);
	memset(backgroundFreq[1].data, 0, len);

	//init weight 0
	ushort* ptr0 = background[0].ptr<ushort>(0);
	for (int i = 0; i < cols; i++)
		ptr0[i] = BGMOG_WEIGHT_USHORT(float(1) / float(CANNY_ORIENTATION_NUM));

	len = 2 * cols;
	for (int i = 1; i < rows; i++)
	{
		ushort* ptr = background[0].ptr<ushort>(i);
		memcpy(ptr, ptr0, len);
	}

	//init weight 1--CANNY_ORIENTATION_NUM
	uchar* data0 = background[0].data;
	len = 2 * rows * cols;
	for (int i = 1; i < CANNY_ORIENTATION_NUM; i++)
	{
		uchar* data = background[i].data;
		memcpy(data, data0, len);
	}

	return;
}

bool edgeBackgroundMOG::sortBKWeight(uchar index0, ushort* w)
{
	CV_Assert(index0 < CANNY_ORIENTATION_NUM);

	ushort ww = w[index0];
	ushort sumW = 0;
	for (int i = 0; i < CANNY_ORIENTATION_NUM; i++)
	{
		if (i != index0 && w[i] > ww)
		{
			sumW += w[i];
			if (sumW >= thresholdU)
				return true;
		}
	}

	return false;
}

void edgeBackgroundMOG::getForeground(const Mat& cannyEdges, Mat& fgMask)
{
	CV_Assert(cannyEdges.depth() == CV_8U); // 8位无符号
	int rows = cannyEdges.rows;
	int cols = cannyEdges.cols;
	if (background[0].rows != rows || background[0].cols != cols)
		initBackground(rows, cols);

	fgMask.create(rows, cols, CV_8U);

	//pointers
	const uchar* edgePtr = cannyEdges.data;
	uchar* fgPtr = fgMask.data;

	ushort* wPtr[CANNY_ORIENTATION_NUM];
	for (int k = 0; k < CANNY_ORIENTATION_NUM; k++)
		wPtr[k] = (ushort*)(background[k].data);

	//foreground
	for (int i = 0; i<rows*cols; i++)
	{
		uchar k0 = edgePtr[i];
		CV_Assert(k0 < CANNY_ORIENTATION_NUM);

		ushort w[CANNY_ORIENTATION_NUM];
		for (int k = 0; k < CANNY_ORIENTATION_NUM; k++)
			w[k] = wPtr[k][i];

		if (k0)
			fgPtr[i] = (sortBKWeight(k0, w)) ? BGMOG_FOREGROUND_MASK : 0;
		else
			fgPtr[i] = 0;
	}

	return;
}

void edgeBackgroundMOG::updateBackground(const Mat& cannyEdges)
{
	CV_Assert(cannyEdges.depth() == CV_8U); // 8位无符号
	int rows = cannyEdges.rows;
	int cols = cannyEdges.cols;
	if (background[0].rows != rows || background[0].cols != cols)
		initBackground(rows, cols);

	//pointers
	const uchar* edgePtr = cannyEdges.data;

	uchar* freqPtr[2];
	freqPtr[0] = backgroundFreq[0].data;
	freqPtr[1] = backgroundFreq[1].data;

	ushort* wPtr[CANNY_ORIENTATION_NUM];
	for (int k = 0; k < CANNY_ORIENTATION_NUM; k++)
		wPtr[k] = (ushort*)(background[k].data);

	//update
	for (int i = 0; i<rows*cols; i++)
	{
		CV_Assert(edgePtr[i] < CANNY_ORIENTATION_NUM);
		uchar k0 = edgePtr[i];

		int freqId = k0 ? 1 : 0;
		if (!freqPtr[freqId][i])
		{
			unsigned w[CANNY_ORIENTATION_NUM];
			unsigned sumW = 0;
			for (uchar k = 0; k < CANNY_ORIENTATION_NUM; k++)
			{
				w[k] = unsigned(wPtr[k][i]);
				sumW += w[k];
			}

			w[k0] += d_alpha;
			float sumF = float(sumW + d_alpha);

			//normalization
			for (uchar k = 0; k < CANNY_ORIENTATION_NUM; k++)
				wPtr[k][i] = BGMOG_WEIGHT_USHORT(float(w[k]) / sumF);
		}

		freqPtr[freqId][i] = (freqPtr[freqId][i] + 1) % freq[freqId];
	}

	return;
}
