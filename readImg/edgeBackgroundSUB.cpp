#include "stdafx.h"
#include "edgeBackgroundSUB.h"

edgeBackgroundSUB::edgeBackgroundSUB()
{
	setParameters(BG_SUB_DEFAULT_INTERVAL, BG_SUB_DEFAULT_CHANGE_A, BG_SUB_DEFAULT_CHANGE_R);

	backgroundLast = 0;
	background_num = 0;

	fgNum = -1;
	interval = -1;
}

edgeBackgroundSUB::edgeBackgroundSUB(int interval, int max_change_a, double max_change_r)
{
	setParameters(interval, max_change_a, max_change_r);

	backgroundLast = 0;
	background_num = 0;

	fgNum = -1;
	interval = -1;
}

edgeBackgroundSUB::~edgeBackgroundSUB()
{
	releaseData();
}

void edgeBackgroundSUB::releaseData()
{
	for (int i = 0; i < background_num; i++)
		background[i].release();

	return;
}

void edgeBackgroundSUB::getParameters(int& interval, int& max_change_a, double& max_change_r)
{
	interval = background_interval;

	max_change_a = foreground_max_change_a;
	max_change_r = foreground_max_change_r;

	return;
}

void edgeBackgroundSUB::setParameters(int interval, int max_change_a, double max_change_r)
{
	if (interval < 1)
		background_interval = 1;
	else
		background_interval = interval;

	foreground_max_change_a = max_change_a;
	foreground_max_change_r = max_change_r;

	return;
}

void edgeBackgroundSUB::reset()
{
	backgroundLast = 0;
	background_num = 0;

	fgNum = -1;
	interval = -1;

	return;
}

void edgeBackgroundSUB::initBackground(int rows, int cols)
{
	//release old data
	releaseData();

	//create new data
	if (rows == 0 || cols == 0)
		return;

	for (int i = 0; i < BG_SUB_FRAME_NUM; i++)
	{
		background[i].create(rows, cols, CV_8U);
		backgroundData[i] = background[i].data;
	}

	backgroundLast = 0;
	background_num = 0;

	fgNum = -1;
	interval = -1;

	return;
}

bool edgeBackgroundSUB::getForeground(const Mat& cannyEdges, Mat& fgMask)
{
	//CV_Assert(cannyEdges.depth() == CV_8U); // 8Î»ÎÞ·ûºÅ
	int rows = cannyEdges.rows;
	int cols = cannyEdges.cols;
	if (background[0].rows != rows || background[0].cols != cols)
		initBackground(rows, cols);
	
	size_t len = rows * cols;

	fgMask.create(rows, cols, CV_8U);
	uchar*  fgData = fgMask.data;
	memset(fgData, 0, len);

	//buffering
	if (background_num < BG_SUB_FRAME_NUM)
		return false;

	//check
	int num = 0;
	const uchar* edgeData = cannyEdges.data;
	for (int i = 0; i < len; i++)
	{
		uchar edge = edgeData[i];
		if (edge)
		{
			int j = 0;
			for (; j < BG_SUB_FRAME_NUM; j++)
			{
				//if ( (edge && !(backgroundData[j][i])) || (!edge && backgroundData[j][i]) )
				//if (edge && (backgroundData[j])[i])
				if (edge == (backgroundData[j])[i])
					break;
			}

			//if (j < BG_SUB_FRAME_NUM)
			if (j >= BG_SUB_FRAME_NUM)
			{
				fgData[i] = BG_SUB_FOREGROUND_MASK;
				num++;
			}
		}
	}

	//detect background switch
	if (fgNum > 0)
	{
		if (num > fgNum)
		{
			int dNumA = num - fgNum;
			double dNumR = double(dNumA) / double(num);

			if (dNumA > foreground_max_change_a && dNumR > foreground_max_change_r)
			{
				reset();
				return false;
			}
		}
	}

	fgNum = num;

	return true; 
}

void edgeBackgroundSUB::updateBackground(const Mat& cannyEdges)
{
	//CV_Assert(cannyEdges.channels() == 1);

	interval++;
	if (interval)
	{
		if (interval < background_interval)
			return;

		interval = 0;
	}

	//buffering
	if (background_num < BG_SUB_FRAME_NUM)
	{
		memcpy(background[background_num].data, cannyEdges.data, cannyEdges.rows * cannyEdges.cols);
		background_num++;

		return;
	}

	//update
	memcpy(background[backgroundLast].data, cannyEdges.data, cannyEdges.rows * cannyEdges.cols);
	backgroundLast = (backgroundLast + 1) % BG_SUB_FRAME_NUM;

	return;
}
