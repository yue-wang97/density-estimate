#pragma once
#include "opencv2/imgproc.hpp"
#include <opencv2\opencv.hpp>

using namespace std;
using namespace cv;

// link; 76543210:  76:childDy  54:childDx, 32:parentDy 10:parentDx  Dy,Dx: 00-invalid, Dxy=(-1,0,1)+2
#define Parent(link) ( (link) & 0x0f )
#define Child(link)  ( (link) & 0xf0 )
#define isRoot(mask, link) ( (mask) && !Parent((link)) )
#define isLeaf(mask, link) ( (mask) && !Child((link)) )

#define clearParent(link) ( (link) = (link) & 0xf0 )
#define clearChild(link)  ( (link) = (link) & 0x0f )
#define setParent(link, dy, dx) ( (link) |= (uchar((dy) + 2)) << 2 | uchar((dx) + 2) )
#define setChild(link, dy, dx)  ( (link) |= (uchar((dy) + 2)) << 6 | (uchar((dx) + 2)) << 4 )
#define getParentDYX(link, dy, dx) ( (dy)=(int(((link) & 0x0c) >> 2) - 2), (dx)=(int((link) & 0x03)     - 2) )
#define getChildDYX(link, dy, dx)   ( (dy)=(int(((link) & 0xc0) >> 6) - 2), (dx)=(int(((link) & 0x30) >> 4) - 2) )

#define MAX_EDGE_NUM				2048
//#define MAX_EDGE_NUM				3 //?????????????
#define MAX_CONNECT_LEN				10
#define MAX_IMAGE_HEIGHT			2000

typedef struct {
	uchar status;
	uchar trackedNum;

	Point startPoint;
	Point currentPoint;

	uchar edgeCorrelated; //used for edge correlation
}EdgeTrackingInfo;

typedef struct {
	ushort y0, x0; //start point
	ushort y1, x1; //end point
	ushort minY, maxY;
	ushort minX, maxX;
}EDGE;

typedef struct {
	ushort edgeNum;
	EDGE edge[MAX_EDGE_NUM];

	Mat greyImg; //grey image, type:uchar
	Mat link;  //type:uchar 
	Mat map;   //edge map, type:ushort, map[*]>=1, it can be used as the mask of body edges; (map[*]-1) is the edgeId

	//for edge tracking
	Mat eigenVal; //eigen value, type:float
	Mat localMax; //localMax, type:uchar
	EdgeTrackingInfo trackingInfo[MAX_EDGE_NUM];

	//for pedestrian detection
	uchar bodyEdgeTag[MAX_EDGE_NUM];  // 0: non body edge; 1: a possible body edge 2: a reliable body edge
}EDGEMAP;

class bodyFilter
{
private:
	void topDownScan(const int rows, const int cols, uchar* mask[], uchar* link[]);
	void bottomUpScan(const int rows, const int cols, uchar* mask[], uchar* link[], int len);

	void leftRightScan(const int rows, const int cols, uchar* mask[], uchar* link[]);
	void rightLeftScan(const int rows, const int cols, uchar* mask[], uchar* link[], int len);

	void getConnectDyDx(Mat& link, EDGE& edge, int* dy, int* dx, int len);

	void mergeEdges(EDGEMAP& edgeMap, ushort edgeId0, ushort edgeId1, int* bridgeDy, int* bridgeDx, int bridegLen);
	void clearEdge(EDGEMAP& edgeMap, ushort edgeId);
	void clearEdgeMap(EDGEMAP& edgeMap);

public:
	bodyFilter();
	~bodyFilter();

	void dilateEdge(const Mat& edge, const Mat& seed, Mat& mask, int size);

	void vEdges(Mat& mask, Mat& link, int len);
	void hEdges(Mat& mask, Mat& link, int len);

	void bodyFilter::getEdgeMask(const Mat& mask, const Mat& link, Mat& bodyEdgeMask);
	void getEdgeMap(const Mat& greyImg, const Mat& edges, const Mat& mask, const Mat& link, EDGEMAP& edgeMap);
	int showEdgeMap(const String name, EDGEMAP& edgeMap, bool save);

	ushort splitEdge(EDGEMAP& edgeMap, ushort edgeId, const int y, const int x);
	void smoothEdgeMap(EDGEMAP& edgeMap);
	void connectVEdgeMap(EDGEMAP& edgeMap, int len);
	void connectHEdgeMap(EDGEMAP& edgeMap, int len);

	void removeRedundantEdge(EDGEMAP& edgeMap);
};
