#pragma once
#include "opencv2/imgproc.hpp"
#include <opencv2\opencv.hpp>

#include "MyCanny.h"
#include "edgeBackgroundSUB.h"
#include "bodyFilter.h"
#include "edgeTracking.h"

using namespace std;
using namespace cv;

#define DEFAULT_BODY_IMAGE_FILTER_SIZE		3
#define DEFAULT_BODY_EDGE_DILATE_SIZE		3
#define DEFAULT_BODY_VERTICAL_EDGE_SIZE		9
#define DEFAULT_BODY_VERTICAL_CONNECT_LEN	2

#define DEFAULT_PEDESTRIAN_ALPHA			0.997
#define DEFAULT_PEDESTRIAN_BETA				-17.6

#define DIR_SAVE (String("testImage\\"))

#define PEDESTRIAN_BUFFER_SIZE				20

typedef struct {
	//canny
	double canny_th1;
	double canny_th2;

	//edge background modelling
	int background_interval;
	int foreground_max_change_a; 
	double foreground_max_change_r;

	//body filter
	int body_image_filter_size;
	int body_edge_dilate_size;
	int body_vertical_edge_size;
	int body_vertical_connect_len;

	//edge tracking
	TrackingParameters 	trackingParameters;

	//pedestrian estimation
	double pedestrianAlpha, pedestrianBeta;
}PedestrianParameters;

class PedestrianDetection
{
private:
	int body_image_filter_size;
	int body_edge_dilate_size;
	int body_vertical_edge_size;
	int body_vertical_connect_len;

	double pedestrianAlpha;
	double pedestrianBeta;

	MyCanny myCanny;
	edgeBackgroundSUB edgeBackgroundSUB;
	bodyFilter bodys;
	edgeTracking edgeTracking;

	Mat movingAreaMask;
	int pedestrian_frame_num, pedestrian_frame_last;
	int pedestrianEdgeNum[PEDESTRIAN_BUFFER_SIZE], pedestrianAreaSize[PEDESTRIAN_BUFFER_SIZE];

public:
	PedestrianDetection();
	~PedestrianDetection();

	void showMask(String name, Mat& colorImg, Mat& mask, bool save);

	void getParameters(PedestrianParameters& parameters);
	void setParameters(PedestrianParameters& parameters);

	void inputFrame(Mat& frame);
	bool getResult(int& pedestrianNum, int& pedestrianArea, Mat& pedestrianAreaMask);
};

