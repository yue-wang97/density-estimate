#pragma once
#include "opencv2/imgproc.hpp"
#include <opencv2\opencv.hpp>

using namespace std;
using namespace cv;

#define CANNY_THRESHOLD1				80
#define CANNY_THRESHOLD2				150

#define CANNY_ORIENTATION_NUM			9	// 0£ºnot an edge point, 1--8: edge orientation 
#define CANNY_TAN1						0.41421356 //tan(22.5)
#define CANNY_TAN2						2.41421356 //tan(67.5)
#define CANNY_ORIENTATION0(dx, dy)		( (dx)<(CANNY_TAN1*(dy)) ? 1: ((dx)<(CANNY_TAN2*(dy)) ? 2 : 3) )
#define CANNY_ORIENTATION1(dx, dy)		( (dy)<(CANNY_TAN1*(dx)) ? 3: ((dy)<(CANNY_TAN2*(dx)) ? 4 : 5) )
#define CANNY_ORIENTATION2(dx, dy)		( (dx)<(CANNY_TAN1*(dy)) ? 5: ((dx)<(CANNY_TAN2*(dy)) ? 6 : 7) )
#define CANNY_ORIENTATION3(dx, dy)		( (dy)<(CANNY_TAN1*(dx)) ? 7: ((dy)<(CANNY_TAN2*(dx)) ? 8 : 1) )
#define CANNY_ORIENTATION(dx, dy)		( (dx)>0 ? ((dy)>0 ? CANNY_ORIENTATION0((dx), (dy)) : CANNY_ORIENTATION1((dx), -(dy))) : ((dy)<0 ? CANNY_ORIENTATION2(-(dx), -(dy)) : CANNY_ORIENTATION3(-(dx), (dy))) )
//#define CANNY_ORIENTATION(dx, dy)		( (dx)>0 ? ( (dy)<0 ? ( -(dy)>(dx) ? uchar(7) : uchar(8) ) : ( (dy)>(dx) ? uchar(2) : uchar(1) ) ) : ( (dy)>0 ? (  -(dx)>(dy) ? uchar(4) : uchar(3) ) : ( (dy)<(dx) ? uchar(6) : uchar(5) ) ) )

typedef struct {
	Point corner;
	uchar size;

	uchar descriptor;
}DescriptorNCCHead;

typedef struct {
	Point corner;
	uchar size;

	uchar descriptor[9 * 9];
}DescriptorNCC9;

typedef struct {
	Point corner;

	ushort len;
	uchar descriptor;
}DescriptorPAIRHead;

typedef struct {
	Point corner;

	ushort len;
	uchar descriptor[1];
}DescriptorPAIR0;

typedef struct {
	Point corner;

	ushort len;
	uchar descriptor[4];
}DescriptorPAIR1;

typedef struct {
	Point corner;

	ushort len;
	uchar descriptor[9];
}DescriptorPAIR2;

typedef struct {
	Point corner;

	ushort len;
	uchar descriptor[15];
}DescriptorPAIR3;

class MyCanny
{
private:
	double th1, th2;

	bool cornerDescriptorExtractor(Mat& greyImg, int bufferSize, DescriptorNCCHead* descriptor);
	bool cornerDescriptorExtractor(Mat& greyImg, int size, int devision, int bufferSize, DescriptorPAIRHead* descriptor);
	double cornerDescriptorDistance(DescriptorNCCHead* descriptor1, DescriptorNCCHead* descriptor2);
	double cornerDescriptorDistance(DescriptorPAIRHead* descriptor1, DescriptorPAIRHead* descriptor2);

public:
	MyCanny();
	MyCanny(double threshold1, double threshold2 = CANNY_THRESHOLD2);
	~MyCanny();
	
	void getParameters(double& threshold1, double& threshold2);
	void setParameters(double threshold1, double threshold2 = CANNY_THRESHOLD2);

	void greyImage(const Mat& image, Mat& grayImg, int filter_size);
	void sobel(const Mat& grayImg, Mat& ori);

	void canny(const Mat& grad_x, const Mat& grad_y, double Th1, double Th2, Mat& edges);
	void cannyGray(const Mat& grayImg, Mat& edges);
	void cannyBGR(const Mat& bgrImg, Mat& edges);

	int featuresLocalMaxEigen(const Mat& image, Mat& eigenVal, Mat& localMax, double qualityLevel, InputArray _mask = noArray(), int blockSize = 3, bool useHarrisDetector = false, double harrisK = 0.04);
	void myGoodFeaturesToTrack(const Mat& eigenVal, const Mat& localMax, vector<Point>& corners, int maxCorners, double minDistance, InputArray _mask = noArray());

	bool cornerDescriptorExtractor(Mat& greyImg, DescriptorNCC9& descriptor);
	bool cornerDescriptorExtractor(Mat& greyImg, int size, DescriptorPAIR0& descriptor);
	bool cornerDescriptorExtractor(Mat& greyImg, int size, DescriptorPAIR1& descriptor);
	bool cornerDescriptorExtractor(Mat& greyImg, int size, DescriptorPAIR2& descriptor);
	bool cornerDescriptorExtractor(Mat& greyImg, int size, DescriptorPAIR3& descriptor);

	double cornerDescriptorDistance(DescriptorNCC9& descriptor1, DescriptorNCC9& descriptor2);
	double cornerDescriptorDistance(DescriptorPAIR0& descriptor1, DescriptorPAIR0& descriptor2);
	double cornerDescriptorDistance(DescriptorPAIR1& descriptor1, DescriptorPAIR1& descriptor2);
	double cornerDescriptorDistance(DescriptorPAIR2& descriptor1, DescriptorPAIR2& descriptor2);
	double cornerDescriptorDistance(DescriptorPAIR3& descriptor1, DescriptorPAIR3& descriptor2);
};
