#include "stdafx.h"
#include "PedestrianDetection.h"

/////////////////////////////////////////????????????
void setTestCannyFg(Mat& cannyEdges, Mat& fgMask)
{
	int rows = cannyEdges.rows;
	int cols = cannyEdges.cols;

	uchar* cannyPtr = cannyEdges.data;
	memset(cannyPtr, 0, rows*cols);
	uchar* fgPtr = fgMask.data;
	memset(fgPtr, 0, rows*cols);

	static int y0 = 150;
	static int x0 = 80;
	int LEN0 = 10;

	static int y1 = 253;
	static int x1 = 81;
	int LEN1 = 40;

	//static int y2 = y0 + LEN0 + 3;
	//static int x2 = 79;
	//int LEN1 = 15;
	static int y2 = 100;
	static int x2 = 50;
	int LEN2 = 50;

	static int y3 = 30;
	static int x3 = 10;
	int LEN3 = 30;

	for (int y = 0; y < rows; y++)
	{
		for (int x = 0; x < cols; x++)
		{
			if ((x == x0) && (y >= y0 && y <= y0 + LEN0))
			{
				fgPtr[x] = 1;
				cannyPtr[x] = 1;
			}

			/*
			if ((x == x1) && (y >= y1 && y <= y1 + LEN1))
			{
				fgPtr[x] = 1;
				cannyPtr[x] = 1;
			}
			*/
		}

		cannyPtr += cols;
		fgPtr += cols;
	}

	return;


	//y0 = y0 + LEN0 + 1; x0--;
	y0 = y0 + LEN0 + 1; x0--;
	fgPtr = fgMask.ptr<uchar>(y0);
	cannyPtr = cannyEdges.ptr<uchar>(y0);
	fgPtr[x0] = 1; cannyPtr[x0] = 1;

	//y0--; x0-- ;
	y0++; x0--;
	fgPtr = fgMask.ptr<uchar>(y0);
	cannyPtr = cannyEdges.ptr<uchar>(y0);
	fgPtr[x0] = 1; cannyPtr[x0] = 1;

	//y0--; x0--;
	y0--; x0--;
	fgPtr = fgMask.ptr<uchar>(y0);
	cannyPtr = cannyEdges.ptr<uchar>(y0);
	fgPtr[x0] = 1; cannyPtr[x0] = 1;

	return;

	/*
	for (int y = 0; y < rows; y++)
	{
	if (y >= 20 && y <= 70)
	{
	cannyPtr[y+10] = 1; fgPtr[y+10] = 1;
	}

	if (y > 29 && y < 49)
	{
	cannyPtr[y + 11] = 1; fgPtr[y + 11] = 1;
	}


	cannyPtr += cols;
	fgPtr += cols;
	}
	return;
	*/

	for (int y = 0; y < rows; y++)
	{
		for (int x = 0; x < cols; x++)
		{
			if ((x == x0) && (y >= y0 && y <= y0 + LEN0))
			{
				cannyPtr[x] = 1;
				fgPtr[x] = 1;
			}

			if ((x == x1) && (y >= y1 && y <= y1 + LEN1))
			{
				cannyPtr[x] = 1;
				fgPtr[x] = 1;
			}

			if ((x == y) && (y >= y3 && y <= y3 + LEN3))
			{
				cannyPtr[x] = 1;
				fgPtr[x] = 1;
			}

			if ((y == y2) && (x >= x2 && x <= x2 + LEN2))
			{
				cannyPtr[x] = 1;
				fgPtr[x] = 1;
			}
		}

		cannyPtr += cols;
		fgPtr += cols;
	}

	y0 += 3;
	x0 += 2;

	y1 += 4;
	x1 += 3;

	y2 += 2;
	x2 += 1;

	y3 += 1;
	x3 += 4;

	return;
}

/////////////////////////////////////////????????????

PedestrianDetection::PedestrianDetection()
{
	body_image_filter_size = DEFAULT_BODY_IMAGE_FILTER_SIZE;
	body_edge_dilate_size = DEFAULT_BODY_EDGE_DILATE_SIZE;
	body_vertical_edge_size = DEFAULT_BODY_VERTICAL_EDGE_SIZE;
	body_vertical_connect_len = DEFAULT_BODY_VERTICAL_CONNECT_LEN;

	pedestrianAlpha = DEFAULT_PEDESTRIAN_ALPHA;
	pedestrianBeta = DEFAULT_PEDESTRIAN_BETA;

	pedestrian_frame_num = 0; 
	pedestrian_frame_last = 0;
}

PedestrianDetection::~PedestrianDetection()
{
}

void PedestrianDetection::getParameters(PedestrianParameters& parameters)
{
	//canny
	myCanny.getParameters(parameters.canny_th1, parameters.canny_th2);

	//edge background modelling
	edgeBackgroundSUB.getParameters(parameters.background_interval, parameters.foreground_max_change_a, parameters.foreground_max_change_r);

	//body filter
	parameters.body_image_filter_size = body_image_filter_size;
	parameters.body_edge_dilate_size = body_edge_dilate_size;
	parameters.body_vertical_edge_size = body_vertical_edge_size;
	parameters.body_vertical_connect_len = body_vertical_connect_len;

	//edge tracking
	edgeTracking.getTrackingParameters(parameters.trackingParameters);

	//pedestrian estimation
	parameters.pedestrianAlpha = pedestrianAlpha;
	parameters.pedestrianBeta = pedestrianBeta;

	return;
}

void PedestrianDetection::setParameters(PedestrianParameters& parameters)
{
	//canny
	myCanny.setParameters(parameters.canny_th1, parameters.canny_th2);

	//edge background modelling
	edgeBackgroundSUB.setParameters(parameters.background_interval, parameters.foreground_max_change_a, parameters.foreground_max_change_r);

	//body filter
	body_image_filter_size = parameters.body_image_filter_size;
	body_edge_dilate_size = parameters.body_edge_dilate_size;
	body_vertical_edge_size = parameters.body_vertical_edge_size;
	body_vertical_connect_len = parameters.body_vertical_connect_len;

	//edge tracking
	edgeTracking.setTrackingParameters(parameters.trackingParameters);

	//pedestrian estimation
	pedestrianAlpha = parameters.pedestrianAlpha;
	pedestrianBeta = parameters.pedestrianBeta;

	return;
}

void PedestrianDetection::showMask(String name, Mat& colorImg, Mat& mask, bool save)
{
	Mat img;
	img = colorImg.clone();

	uchar* imgPtr = img.data;
	uchar* maskPtr = mask.data;
	for (int i = 0; i < mask.rows*mask.cols; i++)
	{
		if (maskPtr[i])
		{
			imgPtr[3 * i] = 255;
			imgPtr[3 * i + 1] = 255;
			imgPtr[3 * i + 2] = 0;
		}
	}

	imshow(name, img);
	if (save)
		imwrite(DIR_SAVE + name + ".bmp", img);

	return;
}

void PedestrianDetection::inputFrame(Mat& frame)
{
	//////// input ////////
	if (frame.empty())
		return;

	//////// grey image ////////
	Mat greyImg;
	myCanny.greyImage(frame, greyImg, body_image_filter_size);
	int rows = greyImg.rows; int cols = greyImg.cols;

/////////////////////////////////////////????????????
//greyImg.setTo(0);
//int xx0 = cols / 2; int yy0 = rows / 2;
//Rect ROI(xx0, yy0, cols - xx0, rows - yy0);
//Mat maskROI = greyImg(ROI);
//maskROI.setTo(200);

//imshow("greyImg", greyImg);
//waitKey();
/////////////////////////////////////////????????????

	//////// canny ////////
	Mat cannyEdges;
	myCanny.cannyGray(greyImg, cannyEdges);
/////////////////////////////////////////????????????
//setTestCannyFg(cannyEdges, fgMask);
//edgeTracking.showGreyMask("cannyGray", greyImg, cannyEdges, true);
//waitKey();
/////////////////////////////////////////????????????


	/*/////// foreground and background////////
	Mat fgMask;
	bool haveFg = edgeBackgroundSUB.getForeground(cannyEdges, fgMask);
	edgeBackgroundSUB.updateBackground(cannyEdges);

	if (!haveFg)
	{
		edgeTracking.reset();
		return;
	}
	*/
/////////////////////////////////////////????????????
//setTestCannyFg(cannyEdges, fgMask);
//edgeTracking.showGreyMask("getForeground", greyImg, fgMask, true);
//waitKey();
/////////////////////////////////////////????????????

	//////// body map ////////
	//(1) dilate
	Mat bodyVEdges;
	//bodys.dilateEdge(cannyEdges, fgMask, bodyVEdges, body_edge_dilate_size);
	cannyEdges.copyTo(bodyVEdges);

	//(2) vertical edges and their mask
	Mat bodyVLink;
	bodys.vEdges(bodyVEdges, bodyVLink, body_vertical_edge_size);

	Mat bodyVEdgeMask;
	bodys.getEdgeMask(bodyVEdges, bodyVLink, bodyVEdgeMask);
/////////////////////////////////////////????????????
//edgeTracking.showGreyMask("getEdgeMask", greyImg, bodyVEdgeMask, true);
//waitKey();
/////////////////////////////////////////????????????


	edgeTracking.newGreyImage(greyImg, bodyVEdgeMask);

	return;
}

bool PedestrianDetection::getResult(int& pedestrianNum, int& pedestrianArea, Mat& pedestrianAreaMask)
{
	if (pedestrian_frame_num == 0)
		return false;

	//sort
	sort(pedestrianEdgeNum,  pedestrianEdgeNum  + pedestrian_frame_num);
	sort(pedestrianAreaSize, pedestrianAreaSize + pedestrian_frame_num);

	//results
	pedestrianNum = int(0.5 + double(pedestrianEdgeNum[pedestrian_frame_num / 2]) * (pedestrianAlpha) + (pedestrianBeta));
	if (pedestrianNum < 0)
		pedestrianNum = 0;

	pedestrianArea = pedestrianAreaSize[pedestrian_frame_num / 2];

	pedestrianAreaMask = movingAreaMask;

	//reset
	pedestrian_frame_num = 0;
	pedestrian_frame_last = 0;

	return true;
}
