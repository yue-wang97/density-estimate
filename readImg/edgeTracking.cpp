#include "stdafx.h"
#include "edgeTracking.h"

edgeTracking::edgeTracking()
{
	TrackingParameters para;

	para.edgeEffectSize = DEFAULT_CORNER_EFFECT_SIZE;

	para.cornerMaxNum = DEFAULT_CORNER_MAX_NUM;
	para.cornerQualityLevel = DEFAULT_CORNER_QUALITY_LEVEL;
	para.cornerMinDistance = DEFAULT_CORNER_MIN_DISTANCE;
	para.cornerMaxMissNum = DEFAULT_CORNER_MAX_MISS_NUM;
	para.cornerMaxTrackingError = DEFAULT_CORNER_MAX_TRACKING_ERROR;
	para.cornerMaxScope = DEFAULT_CORNER_MAX_SCOPE;
	para.cornerMinStableLen = DEFAULT_CORNER_MIN_STABLE_LEN;

	para.possibleMaxWHRatio = DEFAULT_POSSIBLE_MAX_WHRATIO;
	para.possibleMinTrackedNum = DEFAULT_POSSIBLE_MIN_TRACKED_NUM;
	para.reliableMinHeight = DEFAULT_RELIABLE_MIN_HEIGHT;
	para.reliableMinTrackedNum = DEFAULT_RELIABLE_MIN_TRACKED_NUM;

	para.possibleDilateSize = DEFAULT_POSSIBLE_DILATE_SIZE;
	para.possibleErodeSize = DEFAULT_POSSIBLE_ERODE_SIZE;
	para.reliableDilateSize = DEFAULT_RELIABLE_DILATE_SIZE;

	setTrackingParameters(para);
}

edgeTracking::~edgeTracking()
{
	releaseData();
}

void edgeTracking::getTrackingParameters(TrackingParameters& para)
{
	para.edgeEffectSize = edgeEffectSize;

	para.cornerMaxNum = cornerMaxNum;
	para.cornerQualityLevel = cornerQualityLevel;
	para.cornerMinDistance = cornerMinDistance;
	para.cornerMaxMissNum = cornerMaxMissNum;
	para.cornerMaxTrackingError = cornerMaxTrackingError;
	para.cornerMaxScope = cornerMaxScope;
	para.cornerMinStableLen = cornerMinStableLen;

	para.possibleMaxWHRatio = possibleMaxWHRatio;
	para.possibleMinTrackedNum = possibleMinTrackedNum;
	para.reliableMinHeight = reliableMinHeight;
	para.reliableMinTrackedNum = reliableMinTrackedNum;

	para.possibleDilateSize = possibleDilateSize;
	para.possibleErodeSize = possibleErodeSize;
	para.reliableDilateSize = reliableDilateSize;

	return;
}

void edgeTracking::setTrackingParameters(TrackingParameters& para)
{
	edgeEffectSize = para.edgeEffectSize; edgeEffectSize0 = 2 * edgeEffectSize + 1;

	cornerMaxNum = para.cornerMaxNum;
	cornerQualityLevel = para.cornerQualityLevel;
	cornerMinDistance = para.cornerMinDistance;
	cornerMaxMissNum = para.cornerMaxMissNum;
	cornerMaxTrackingError = para.cornerMaxTrackingError;
	cornerMaxScope = para.cornerMaxScope; cornerMaxScope2 = cornerMaxScope * cornerMaxScope;
	cornerMinStableLen = para.cornerMinStableLen;

	possibleMaxWHRatio = para.possibleMaxWHRatio;
	possibleMinTrackedNum = para.possibleMinTrackedNum;
	reliableMinHeight = para.reliableMinHeight;
	reliableMinTrackedNum = para.reliableMinTrackedNum;

	possibleDilateSize = para.possibleDilateSize;
	possibleErodeSize = para.possibleErodeSize;
	reliableDilateSize = para.reliableDilateSize;

	//resize
	pathStatus.resize(cornerMaxNum);
	pathCornerStart.resize(cornerMaxNum);
	pathCornerEnd.resize(cornerMaxNum);
	pathCornerMissNum.resize(cornerMaxNum);
	pathLen.resize(cornerMaxNum);

	//clear data
	clearTrackingData();

	return;
}

void edgeTracking::releaseData()
{
	pathStatus.clear(); pathStatus.swap(vector<uchar>());
	pathCornerStart.clear(); pathCornerStart.swap(vector<Point>());
	pathCornerEnd.clear();	pathCornerEnd.swap(vector<Point2f>());
	pathCornerMissNum.clear(); pathCornerMissNum.swap(vector<uchar>());
	pathLen.clear(); pathLen.swap(vector<uchar>());

	backgroundImg.release();
	backgroundLocalMax.release();
	movingLocalMax.release();

	return;
}

void edgeTracking::clearTrackingData()
{
	pathStatus.clear();
	pathCornerStart.clear();
	pathCornerEnd.clear();
	pathCornerMissNum.clear();
	pathLen.clear();

	imageNum = 0;

	return;
}

void edgeTracking::reset()
{
	clearTrackingData();
	return;
}

void edgeTracking::initBackground(Mat& greyImg)
{
	//create new data
	backgroundImg.release();
	backgroundImg.create(greyImg.size(), greyImg.type());

	backgroundLocalMax.release();
	backgroundLocalMax.create(greyImg.size(), greyImg.type());

	movingLocalMax.release();
	movingLocalMax.create(greyImg.size(), greyImg.type());

	//clear data
	clearTrackingData();

	return;
}

bool edgeTracking::correctTrackedCorner(Point2f& corner, Mat& localMax)
{
	int rows = localMax.rows;
	int cols = localMax.cols;

	int y0 = int(corner.y + 0.5);
	int x0 = int(corner.x + 0.5);

	//search
	int minDis2 = 2 * cornerMaxTrackingError * cornerMaxTrackingError + 1;
	int yy = -1, xx = -1;

	uchar* ptr = (uchar*)(localMax.data) + (y0 - cornerMaxTrackingError - 1) * cols;
	for (int y = y0 - cornerMaxTrackingError; y <= y0 + cornerMaxTrackingError; y++)
	{
		ptr += cols;

		if (y < 0)
			continue;
		if (y >= rows)
			break;

		int dy = y - y0; int dy2 = dy * dy;
		for (int x = x0 - cornerMaxTrackingError; x <= x0 + cornerMaxTrackingError; x++)
		{
			if (x < 0)
				continue;
			if (x >= cols)
				break;

			if (ptr[x])
			{
				int dx = x - x0; 
				int dd = dx * dx + dy2;
				if (dd < minDis2)
				{
					minDis2 = dd;
					yy = y; xx = x;
				}
			}
		}
	}

	if ((yy < 0 || yy >= rows) || (xx < 0 || xx >= cols))
		return false;

	corner = Point2f(float(xx), float(yy));

	return true;
}

int edgeTracking::addNewCorners(Mat& eigenVal, Mat& localMax, Mat& cornerMask)
{
	int rows = localMax.rows;
	int cols = localMax.cols;

	//////add new corners//////
	int cornerNum = int(pathCornerStart.capacity() - pathCornerStart.size());
	if (cornerNum <= 0)
		return 0;

	vector<Point> corners(cornerNum);
	myGoodFeaturesToTrack(eigenVal, localMax, corners, cornerNum, cornerMinDistance, cornerMask);
	
	int numAddedNewCorners = corners.size();
	if (numAddedNewCorners == 0)
		return 0;

	//add 
	vector<Point2f> fCorners(numAddedNewCorners);
	Mat(corners).convertTo(fCorners, Mat(fCorners).type());

	pathStatus.insert(pathStatus.end(), numAddedNewCorners, STATUS_TRACKING_STATIONARY);	//在尾部插入numAddedNewCorners个 PATH_STATUS_UNTRACKED
	pathCornerStart.insert(pathCornerStart.end(), corners.begin(), corners.end());
	pathCornerEnd.insert(pathCornerEnd.end(), fCorners.begin(), fCorners.end());
	pathCornerMissNum.insert(pathCornerMissNum.end(), numAddedNewCorners, 0);	//在尾部插入numAddedNewCorners个0 
	pathLen.insert(pathLen.end(), numAddedNewCorners, 1);	//在尾部插入numAddedNewCorners个1  

	return numAddedNewCorners;
}

int edgeTracking::updateTrackingCorners(Mat& eigenVal, Mat& localMax, Mat& cornerMask)
{
	int rows = localMax.rows;
	int cols = localMax.cols;

	//// update corner mask //// 
	uchar* ptr = (uchar*)(cornerMask.data);
	for (int i = 0; i < pathCornerEnd.size(); i++)
	{
		int y = int(pathCornerEnd[i].y + 0.5);
		int x = int(pathCornerEnd[i].x + 0.5);

		int x0 = x - cornerMinDistance;
		if (x0 >= cols)
			continue;
		if (x0 < 0)
			x0 = 0;
		int y0 = y - cornerMinDistance;
		if (y0 >= rows)
			continue;
		if (y0 < 0)
			y0 = 0;

		int x1 = x + cornerMinDistance;
		if (x1 < 0)
			continue;
		if (x1 >= cols)
			x1 = cols - 1;
		int y1 = y + cornerMinDistance;
		if (y1 < 0)
			continue;
		if (y1 >= rows)
			y1 = rows - 1;

		Rect ROI(x0, y0, x1 - x0 + 1, y1 - y0 + 1);
		Mat cornerMaskROI = cornerMask(ROI);
		cornerMaskROI.setTo(0);
	}

	return  addNewCorners(eigenVal, localMax, cornerMask);
}

void edgeTracking::cornerTracking(Mat& greyImg, Mat& eigenVal, Mat& localMax, Mat& cornerMask)
{
	int rows = greyImg.rows;
	int cols = greyImg.cols;

	Mat movingLocalMax; 
	localMax.copyTo(movingLocalMax); 
	uchar* movingLocalMaxPtr = (uchar*)movingLocalMax.data;

	//tracking by applying optical flow
	int cornerNum = pathCornerEnd.size();

	vector<Point2f> trackedCorners(cornerNum);
	vector<uchar> status(cornerNum);
	vector<float> errors(cornerNum);
	if (cornerNum > 0)
		calcOpticalFlowPyrLK(backgroundImg, greyImg, pathCornerEnd, trackedCorners, status, errors);

	//tracking and verifing tracked corners
	for (int i = 0; i < cornerNum; i++)
	{
		//CV_Assert(pathCornerStart[i].y >= 0 && pathCornerStart[i].y < rows); 
		//CV_Assert(pathCornerStart[i].x >= 0 && pathCornerStart[i].x < cols);

		//is tracked?
		float x = trackedCorners[i].x; float y = trackedCorners[i].y;
		if (status[i] == 0 || x < 0 || x >= cols || y < 0 || y >= rows)
		{
			pathStatus[i] = STATUS_TRACKING_UNTRACKED;
			continue;
		}

		//check if the trackedCorner has a corresponding newCorner?
		if (!correctTrackedCorner(trackedCorners[i], localMax))
			pathCornerMissNum[i]++;
		else
			pathCornerMissNum[i] = 0;

		if (pathCornerMissNum[i] > cornerMaxMissNum)
		{
			pathStatus[i] = STATUS_TRACKING_UNTRACKED;
			continue;
		}

		//has been outscope?
		if (pathStatus[i] == STATUS_TRACKING_OUTSCOPE)
		{
			pathLen[i]++;
			continue;
		}

		//is a real stationary corner?
		int y0 = pathCornerStart[i].y; int x0 = pathCornerStart[i].x;
		if (int(y + 0.5) == y0 && int(x + 0.5) == x0)
		{
			pathStatus[i] = STATUS_TRACKING_UNTRACKED;
			movingLocalMaxPtr[y0 * cols + x0] = 0;
			continue;
		}

		//is outscope?
		float dy = y - y0; float dx = x - x0;
		float dd = dy * dy + dx * dx;
		if (dd > cornerMaxScope2)
		{
			pathStatus[i] = STATUS_TRACKING_OUTSCOPE;
			pathLen[i]++;
			continue;
		}

		pathLen[i]++;
	}

	//////remove invalid features//////
	int k = 0;
	for (int i = 0; i < pathCornerEnd.size(); i++)
	{
		if (pathStatus[i] == STATUS_TRACKING_UNTRACKED)
			continue;

		pathCornerEnd[k] = trackedCorners[i];

		pathStatus[k] = pathStatus[i];
		pathCornerStart[k] = pathCornerStart[i];
		pathCornerMissNum[k] = pathCornerMissNum[i];
		pathLen[k] = pathLen[i];
		k++;
	}

	pathStatus.resize(k);
	pathCornerStart.resize(k);
	pathCornerEnd.resize(k);
	pathCornerMissNum.resize(k);
	pathLen.resize(k);

	//update tracking corners
	updateTrackingCorners(eigenVal, movingLocalMax, cornerMask);

	return;
}

inline void maskDifference(const Mat& mask0, const Mat& mask1, Mat& mask)
{
	uchar* ptr0 = (uchar*)mask0.data;
	uchar* ptr1 = (uchar*)mask1.data;
	uchar* ptr = (uchar*)mask.data;

	for (int i = 0; i < mask0.rows * mask0.cols; i++)
		ptr[i] = ptr1[i] & (ptr1[i] ^ ptr0[i]);

	return;
}

void edgeTracking::newGreyImage(Mat& greyImg, Mat& edgeMask)
{
	// eigenValue and localMax
	Mat eigenVal, localMax;
	featuresLocalMaxEigen(greyImg, eigenVal, localMax, cornerQualityLevel);

	//init
	if (imageNum == 0)
	{
		initBackground(greyImg);

		localMax.copyTo(backgroundLocalMax);
		imageNum++;

		return;
	}

	//get moving local-Max
	maskDifference(backgroundLocalMax, localMax, movingLocalMax);

	//get corner-Mask
	Mat cornerMask;
	Mat element = getStructuringElement(MORPH_RECT, Size(edgeEffectSize0, edgeEffectSize0));
	dilate(edgeMask, cornerMask, element);
////////////////////////
showGreyMask("edgeMask-dilate", greyImg, cornerMask, true);
//waitKey(0);
///////////////////////

	//tracking
	if (imageNum == 1)
		addNewCorners(eigenVal, movingLocalMax, cornerMask);
	else
		cornerTracking(greyImg, eigenVal, movingLocalMax, cornerMask);

	//save background image
	greyImg.copyTo(backgroundImg);
	localMax.copyTo(backgroundLocalMax);
	imageNum++;

/////////////////////
showTrackedCorners("cornerTracking", greyImg, movingLocalMax, true);
waitKey(1);
/////////////////////

	return;
}

inline void dilateErode(Mat& mask, int dSize, int eSize)
{
	Mat element = getStructuringElement(MORPH_RECT, Size(dSize, dSize));
	dilate(mask, mask, element);

	element = getStructuringElement(MORPH_RECT, Size(eSize, eSize));
	erode(mask, mask, element);
}

void edgeTracking::bodyEdgeDetect(EDGEMAP& edgeMap)
{
	for (ushort edgeId = 1; edgeId <= edgeMap.edgeNum; edgeId++)
	{
		uchar bodyEdgeTag = edgeMap.bodyEdgeTag[edgeId - 1];
		clearBodyEdge(bodyEdgeTag);

		EDGE& edge = edgeMap.edge[edgeId - 1];
		int height = edge.maxY - edge.minY;
		int width = edge.maxX - edge.minX;

		//====possible edge====
		//HV ratio test
		if ((double(width) / double(height)) > possibleMaxWHRatio)
			continue;

		EdgeTrackingInfo& trackingInfo = edgeMap.trackingInfo[edgeId - 1];
		if (trackingInfo.trackedNum < possibleMinTrackedNum)
			continue;

		setPossibleBodyEdge(bodyEdgeTag);

		//====reliable edge====
		if (height < reliableMinHeight)
			continue;
		if (trackingInfo.trackedNum < reliableMinTrackedNum)
			continue;

		setReliableBodyEdge(bodyEdgeTag);
	}

	return;
}

int edgeTracking::showMovingBodyArea(const String name, Mat& image, Mat& movingAreaMask)
{
	int rows = image.rows;
	int cols = image.cols;

	int areaSize = 0;

	uchar* imagePtr = image.data;
	uchar* areaPtr = movingAreaMask.data;
	for (int i = 0; i < rows * cols; i++)
	{
		if (areaPtr[i])
		{
			imagePtr[3 * i] = 255;
			imagePtr[3 * i + 1] = 255;

			areaSize++;
		}
	}

	imshow(name, image);

	return areaSize;
}

bool edgeTracking::movingBodyArea(EDGEMAP& edgeMap, int& edgeNum, int& areaSize, Mat& movingAreaMask)
{
	edgeNum = 0; areaSize = 0;

	if (edgeMap.edgeNum == 0)
		return false;

	Mat& link = edgeMap.link;
	int rows = link.rows;
	int cols = link.cols;
	size_t len = rows * cols;

	//init
	if (movingAreaMask.rows != rows || movingAreaMask.cols != cols)
		movingAreaMask.create(rows, cols, CV_8U);
	memset(movingAreaMask.data, 0, len); // mask 0

	//edge mask
	for (ushort edgeId = 1; edgeId <= edgeMap.edgeNum; edgeId++)
	{
		uchar bodyEdgeTag = edgeMap.bodyEdgeTag[edgeId - 1];
		if (isNotPossibleBodyEdge(bodyEdgeTag))
			continue;

		EDGE& edge = edgeMap.edge[edgeId - 1];
		int y = edge.y0; int x = edge.x0;

		uchar* linkPtr = link.ptr<uchar>(y);
		uchar* areaPtr = movingAreaMask.ptr<uchar>(y);
		while (1)
		{
			areaPtr[x] = 1; // mask 1

			uchar child = Child(linkPtr[x]);
			if (!child)
				break;

			int dy, dx;
			getChildDYX(child, dy, dx);

			y += dy; x += dx;
			//CV_Assert((y >= 0 && y < rows) && (x >= 0 && x < cols));

			if (dy == -1)
			{
				linkPtr -= cols; areaPtr -= cols;
			}
			else
			{
				if (dy) //dy==1
				{
					linkPtr += cols; areaPtr += cols;
				}
			}
		}
	}

	//poddible edge dilation and erosion
	dilateErode(movingAreaMask, possibleDilateSize, possibleErodeSize);

	//reliable edge dilation
	Mat reliableEdgeArea(rows, cols, CV_8U);
	memset(reliableEdgeArea.data, 0, len); // mask 0

	for (ushort edgeId = 1; edgeId <= edgeMap.edgeNum; edgeId++)
	{
		uchar bodyEdgeTag = edgeMap.bodyEdgeTag[edgeId - 1];
		if (isNotReliableBodyEdge(bodyEdgeTag))
			continue;
		edgeNum++;

		EDGE& edge = edgeMap.edge[edgeId - 1];
		int y = edge.y0; int x = edge.x0;

		uchar* linkPtr = link.ptr<uchar>(y);
		uchar* reliableEdgePtr = reliableEdgeArea.ptr<uchar>(y);
		while (1)
		{
			reliableEdgePtr[x] = 1; // mask 1

			uchar child = Child(linkPtr[x]);
			if (!child)
				break;

			int dy, dx;
			getChildDYX(child, dy, dx);

			y += dy; x += dx;
			//CV_Assert((y >= 0 && y < rows) && (x >= 0 && x < cols));

			if (dy == -1)
			{
				linkPtr -= cols; reliableEdgePtr -= cols;
			}
			else
			{
				if (dy) //dy==1
				{
					linkPtr += cols; reliableEdgePtr += cols;
				}
			}
		}
	}

	Mat element = getStructuringElement(MORPH_RECT, Size(reliableDilateSize, reliableDilateSize));
	dilate(reliableEdgeArea, reliableEdgeArea, element);

	//mask
	uchar* areaPtr = movingAreaMask.data;
	uchar* reliableEdgePtr = reliableEdgeArea.data;
	int areaSize0 = 0;
	for (int i = 0; i < len; i++)
	{
		if (areaPtr[i] && reliableEdgePtr[i])
		{
			areaPtr[i] = MOVING_AREA_MASK;
			areaSize0++;
		}
		else
			areaPtr[i] = 0;
	}

	areaSize = areaSize0;

	return true;
}

////////////////////////////////////////////
void edgeTracking::showGreyMask(String name, Mat& greyImg, Mat& mask, bool save)
{
	CV_Assert(mask.depth() == CV_8U); // 8位无符号  

	Mat img;
	cvtColor(greyImg, img, CV_GRAY2BGR);

	uchar* imgPtr = img.data;
	uchar* maskPtr = mask.data;
	for (int i = 0; i < mask.rows*mask.cols; i++)
	{
		if (maskPtr[i])
		{
			imgPtr[3 * i] = 0;
			imgPtr[3 * i + 1] = 255;
			imgPtr[3 * i + 2] = 0;
		}
	}

	imshow(name, img);
	if (save)
		imwrite("testImage\\" + name + ".bmp", img);

	return;
}

int edgeTracking::showEdgeMapTracking(const String name, EDGEMAP& edgeMap, int trackedNum, bool showImpossibleEdges, bool showReliableEdges, bool save)
{
	ushort edgesNum = edgeMap.edgeNum;
	EDGE* edges = edgeMap.edge;
	EdgeTrackingInfo* trackingInfo = edgeMap.trackingInfo;
	uchar* bodyEdgeTag = edgeMap.bodyEdgeTag;

	Mat image;
	cvtColor(edgeMap.greyImg, image, CV_GRAY2BGR);
	Mat& link = edgeMap.link;
	Mat& map = edgeMap.map;
	Mat& eigenVal = edgeMap.eigenVal;
	Mat& localMax = edgeMap.localMax;

	int rows = link.rows;
	int cols = link.cols;

//*/////////////////////
uchar* imgPtr = image.data;
uchar* maskPtr = localMax.data;
for (int i = 0; i < rows * cols; i++)
{
	if (maskPtr[i])
	{
		imgPtr[3 * i] = 0;
		imgPtr[3 * i + 1] = 255;
		imgPtr[3 * i + 2] = 255;
	}
}
/////////////////////////*/

	uchar edgeShown[MAX_EDGE_NUM];
	memset(edgeShown, 0, MAX_EDGE_NUM);

	int shownEdgeNum = 0;

	uchar* imagePtr = image.data;
	uchar* linkPtr = link.data;
	ushort* mapPtr = (ushort*)(map.data);
	float* eigenValPtr = (float*)(eigenVal.data);
	uchar* localMaxPtr = localMax.data;
	for (ushort y0 = 0; y0 < rows; y0++)
	{
		for (ushort x0 = 0; x0 < cols; x0++)
		{
			if (mapPtr[x0])
				CV_Assert(mapPtr[x0] > 0 && mapPtr[x0] <= edgesNum);

			if (isRoot(mapPtr[x0], linkPtr[x0]))
			{
				ushort edgeId = mapPtr[x0];
				CV_Assert(!edgeShown[edgeId - 1]);
				edgeShown[edgeId - 1] = 1;

				EDGE& edge = edges[edgeId - 1];
				CV_Assert(edge.y0 == y0 && edge.x0 == x0);

				if (trackingInfo[edgeId - 1].trackedNum < trackedNum)
					continue;
				if (showImpossibleEdges && !isNotPossibleBodyEdge(bodyEdgeTag[edgeId - 1]))
					continue;
				if (showReliableEdges && isNotReliableBodyEdge(bodyEdgeTag[edgeId - 1]))
					continue;
				shownEdgeNum++;

				float val = eigenValPtr[x0];
				if (localMaxPtr[x0] && val > 0.0)
				{
					imagePtr[3 * x0] = 0;
					imagePtr[3 * x0 + 1] = 0;
					imagePtr[3 * x0 + 2] = 255;
				}
				else
				{
					imagePtr[3 * x0] = 0;
					imagePtr[3 * x0 + 1] = 255;
					imagePtr[3 * x0 + 2] = 0;
				}

				int y = y0; int x = x0;
				uchar* imagePtr0 = imagePtr; uchar* linkPtr0 = linkPtr; ushort* mapPtr0 = mapPtr; float* eigenValPtr0 = eigenValPtr; uchar* localMaxPtr0 = localMaxPtr;
				while (1)
				{
					uchar child = Child(linkPtr0[x]);
					if (!child)
						break;

					int dy, dx;
					getChildDYX(child, dy, dx);

					y += dy; x += dx;
					CV_Assert((y >= 0 && y < rows) && (x >= 0 && x < cols));
					CV_Assert((y >= edge.minY && y <= edge.maxY) && (x >= edge.minX && x <= edge.maxX));

					if (dy == -1)
					{
						imagePtr0 -= 3 * cols; linkPtr0 -= cols; mapPtr0 -= cols; eigenValPtr0 -= cols; localMaxPtr0 -= cols;
					}
					else
					{
						if (dy) //dy==1
						{
							imagePtr0 += 3 * cols; linkPtr0 += cols; mapPtr0 += cols; eigenValPtr0 += cols; localMaxPtr0 += cols;
						}
					}

					uchar parent = Parent(linkPtr0[x]);
					CV_Assert(parent);
					int dy0, dx0;
					getParentDYX(parent, dy0, dx0);
					CV_Assert(dy0 == -dy && dx0 == -dx);

					CV_Assert(mapPtr0[x] == edgeId);

					float val = eigenValPtr0[x];
					if (localMaxPtr0[x] && val > 0.0)
					{
						imagePtr0[3 * x] = 0;
						imagePtr0[3 * x + 1] = 0;
						imagePtr0[3 * x + 2] = 255;
					}
					else
					{
						imagePtr0[3 * x] = 0;
						imagePtr0[3 * x + 1] = 255;
						imagePtr0[3 * x + 2] = 0;
					}
				}

				CV_Assert(edge.y1 == y && edge.x1 == x);
			}
		}

		imagePtr += 3 * cols; linkPtr += cols; mapPtr += cols; eigenValPtr += cols;  localMaxPtr += cols;
	}

	for (int i = 0; i < edgesNum; i++)
		CV_Assert(edgeShown[i]);
	CV_Assert(shownEdgeNum == edgesNum);

	imshow(name, image);
	if(save)
		imwrite("testImage\\" + name + ".bmp", image);

	return shownEdgeNum;
}

void edgeTracking::showTrackedCorners(String name, Mat& greyImg, Mat& localMax, bool save)
{
	int rows = greyImg.rows;
	int cols = greyImg.cols;

	Mat colorImg;
	cvtColor(greyImg, colorImg, CV_GRAY2BGR);

	//circles
	int i = 0;
	for (; i < pathCornerEnd.size(); i++)
	{
		if (pathStatus[i] == STATUS_TRACKING_UNTRACKED)
		{
			circle(colorImg, pathCornerEnd[i], 2, Scalar(255, 0, 0), 2);
			continue;
		}

		if (pathStatus[i] == STATUS_TRACKING_STATIONARY)
		{
			circle(colorImg, pathCornerEnd[i], 2, Scalar(0, 255, 0), 2);
			continue;
		}

		if (pathStatus[i] == STATUS_TRACKING_OUTSCOPE)
		{
			circle(colorImg, pathCornerEnd[i], 2, Scalar(0, 0, 255), 2);
			continue;
		}
	}

	//localMax
	uchar* maxPtr = (uchar*)(localMax.data);
	uchar* imgPtr = (uchar*)(colorImg.data);
	for (int y = 0; y < rows; y++)
	{
		for (int x = 0; x < cols; x++)
		{
			if (maxPtr[x])
			{
				imgPtr[3 * x + 0] = 0;
				imgPtr[3 * x + 1] = 0;
				imgPtr[3 * x + 2] = 255;
			}
		}

		maxPtr += cols;
		imgPtr += cols * 3;
	}

	//show and save
	imshow(name, colorImg);
	if (save)
		imwrite("testImage\\" + name + ".bmp", colorImg);

	return;
}

/////////////////????????????//////////////////////
void edgeTracking::testCorrectTrackedCorner()
{
	int rows = 200; int cols = 300;
	Mat localMax;
	localMax.create(rows, cols, CV_8U);
	localMax.setTo(0);
	uchar* maxPtr = (uchar*)(localMax.data);

	int y0 = 199; int x0 = 298;
	Point2f corner = Point2f(x0, y0);

	int y, x;
	y = 3; x = 5;
	maxPtr[300 * y + x] = 1;

	y = 4; x = 6;
	maxPtr[cols * y + x] = 1;

	bool test = correctTrackedCorner(corner, localMax);

	return;
}
