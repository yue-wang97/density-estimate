#pragma once
#include "opencv2/imgproc.hpp"
#include <opencv2\opencv.hpp>

using namespace std;
using namespace cv;

#include "MyCanny.h"
#include "bodyFilter.h"

#define clearBodyEdge(bodyEdgeTag)			(bodyEdgeTag = 0)
#define setPossibleBodyEdge(bodyEdgeTag)	(bodyEdgeTag = 1)
#define setReliableBodyEdge(bodyEdgeTag)	(bodyEdgeTag = 2)
#define isNotPossibleBodyEdge(bodyEdgeTag)	(!bodyEdgeTag)
#define isNotReliableBodyEdge(bodyEdgeTag)	(bodyEdgeTag < 2)

#define DEFAULT_CORNER_EFFECT_SIZE			2 // 2 * DEFAULT_CORNER_EFFECT_SIZE + 1
#define DEFAULT_CORNER_MAX_NUM				500
#define DEFAULT_CORNER_QUALITY_LEVEL		0.03
#define DEFAULT_CORNER_MIN_DISTANCE			10 // 2 * DEFAULT_CORNER_MIN_DISTANCE + 1
#define DEFAULT_CORNER_MAX_TRACKING_ERROR	2
#define DEFAULT_CORNER_MAX_MISS_NUM			3
#define DEFAULT_CORNER_MAX_SCOPE			10
#define DEFAULT_CORNER_MIN_STABLE_LEN		4

#define DEFAULT_POSSIBLE_MAX_WHRATIO		0.70 //tan(30)=0.58; tan(35)=0.70; tan(40)=0.84
#define DEFAULT_POSSIBLE_MIN_TRACKED_NUM	0
#define DEFAULT_RELIABLE_MIN_HEIGHT			12
#define DEFAULT_RELIABLE_MIN_TRACKED_NUM	1

#define DEFAULT_POSSIBLE_DILATE_SIZE		15
#define DEFAULT_POSSIBLE_ERODE_SIZE			12
#define DEFAULT_RELIABLE_DILATE_SIZE		30

#define STATUS_TRACKING_UNTRACKED			0
#define STATUS_TRACKING_STATIONARY			1
#define STATUS_TRACKING_OUTSCOPE			2

#define MOVING_AREA_MASK					1

typedef struct {
	//Parameters for edge tracking
	int edgeEffectSize;

	int cornerMaxNum;
	double cornerQualityLevel;
	int cornerMinDistance;
	int cornerMaxMissNum;
	ushort cornerMaxTrackingError;
	int cornerMaxScope;
	int cornerMinStableLen;

	//Parameters for possible and reliable edge detection
	double possibleMaxWHRatio;
	int possibleMinTrackedNum;

	int reliableMinHeight, reliableMinTrackedNum;

	//Parameters for detection of moving body areas
	int possibleDilateSize, possibleErodeSize;
	int reliableDilateSize;
}TrackingParameters;

class edgeTracking : public MyCanny, bodyFilter
{
private:
	//parameters
	int edgeEffectSize, edgeEffectSize0;
	int cornerMaxNum;
	double cornerQualityLevel;
	int cornerMinDistance;
	int cornerMaxMissNum;
	ushort cornerMaxTrackingError;
	int cornerMaxScope, cornerMaxScope2;
	int cornerMinStableLen;

	double possibleMaxWHRatio;
	int possibleMinTrackedNum;
	int reliableMinHeight, reliableMinTrackedNum;

	int possibleDilateSize, possibleErodeSize;
	int reliableDilateSize;

	//data
	vector<uchar> pathStatus;
	vector<Point> pathCornerStart;
	vector<Point2f> pathCornerEnd;
	vector<uchar> pathCornerMissNum;
	vector<uchar> pathLen;

	Mat backgroundImg;
	Mat backgroundLocalMax;
	Mat movingLocalMax;
	unsigned int imageNum;

	//functions
	void releaseData();
	void clearTrackingData();
	void initBackground(Mat& greyImg);

	bool correctTrackedCorner(Point2f& corner, Mat& localMax);
	int addNewCorners(Mat& eigenVal, Mat& localMax, Mat& cornerMask);
	int updateTrackingCorners(Mat& eigenVal, Mat& localMax, Mat& cornerMask);
	void cornerTracking(Mat& greyImg, Mat& eigenVal, Mat& localMax, Mat& cornerMask);

public:
	edgeTracking();
	~edgeTracking();

	void getTrackingParameters(TrackingParameters& para);
	void setTrackingParameters(TrackingParameters& para);

	void reset();
	void newGreyImage(Mat& greyImg, Mat& edgeMask);

	void bodyEdgeDetect(EDGEMAP& edgeMap);

	int showMovingBodyArea(const String name, Mat& image, Mat& movingAreaMask);
	bool movingBodyArea(EDGEMAP& edgeMap, int& edgeNum, int& areaSize, Mat& movingAreaMask);






	///???????////
	void showGreyMask(String name, Mat& greyImg, Mat& mask, bool save);
	int showEdgeMapTracking(const String name, EDGEMAP& edgeMap, int trackedNum, bool showImpossibleEdges, bool showReliableEdges, bool save);
	void showTrackedCorners(String name, Mat& greyImg, Mat& localMax, bool save);

	void edgeTracking::testCorrectTrackedCorner();
};

