#include "stdafx.h"
#include <string.h>
#include <stdio.h>

#include "MyCanny.h"

MyCanny::MyCanny()
{
	setParameters(CANNY_THRESHOLD1, CANNY_THRESHOLD2);
}

MyCanny::MyCanny(double threshold1, double threshold2)
{
	setParameters(threshold1, threshold2);
}

MyCanny::~MyCanny()
{
}

void MyCanny::getParameters(double& threshold1, double& threshold2)
{
	threshold1 = th1;
	threshold2 = th2;

	return;
}

void MyCanny::setParameters(double threshold1, double threshold2)
{
	th1 = threshold1;
	th2 = threshold2;

	return;
}

void MyCanny::greyImage(const Mat& image, Mat& grayImg, int filter_size)
{
	size_t len = grayImg.rows*grayImg.cols;

	Mat grayImg0;
	cvtColor(image, grayImg0, CV_BGR2GRAY);

	/*
	Mat floatImg;
	floatImg.create(grayImg0.rows, grayImg0.cols, CV_32F);

	uchar* grayPtr = grayImg0.data;
	float* floatPtr = (float*)(floatImg.data);

	uchar min = 255, max = 0;
	for (int i = 0; i < len; i++)
	{
	if (min > grayPtr[i])
	min = grayPtr[i];
	if (max < grayPtr[i])
	max = grayPtr[i];
	}
	if (max == min)
	memset(grayPtr, 0, len);
	else
	{
	float 	dd = max - min;
	for (int i = 0; i < len; i++)
	grayPtr[i] = uchar(float(grayPtr[i] - min) * 255.0 / dd);
	}

	float dd, min = 255.0, max = 0.0;
	for (int i = 0; i < len; i++)
	{
		dd = sqrtf(float(grayPtr[i]));
		floatPtr[i] = dd;

		if (min > dd)
			min = dd;
		if (max < dd)
			max = dd;
	}

	dd = max - min;
	if (dd < 0.001)
		memset(grayPtr, 0, len);
	else
	{
		for (int i = 0; i < len; i++)
			grayPtr[i] = uchar((floatPtr[i] - min) * 255 / dd);
	}
	*/

	//filtering 
	GaussianBlur(grayImg0, grayImg, Size(filter_size, filter_size), 0, 0, BORDER_DEFAULT);
	//bilateralFilter(grayImg0, grayImg, filter_size, double(filter_size)*2.0, double(filter_size)/2.0);

	return;
}

void MyCanny::sobel(const Mat& grayImg, Mat& ori)
{
	//CV_Assert(grayImg.channels() == 1);

	//Sobel Gradient, 梯度图像的深度，设定为 CV_16S 避免外溢。 
	Mat grad_x, grad_y;
	Sobel(grayImg, grad_x, CV_16S, 1, 0, 3, 1, 0, BORDER_REPLICATE);
	Sobel(grayImg, grad_y, CV_16S, 0, 1, 3, 1, 0, BORDER_REPLICATE);

	//Orientation
	int rows = grayImg.rows;
	int cols = grayImg.cols;
	ori.create(rows, cols, CV_8U);

	uchar* oriPtr = ori.data;
	short* grad_xPtr = (short*)(grad_x.data);
	short* grad_yPtr = (short*)(grad_y.data);
	for (int i = 0; i<rows*cols; i++)
		oriPtr[i] = CANNY_ORIENTATION(grad_xPtr[i], grad_yPtr[i]);

	return;
}

void MyCanny::canny(const Mat& grad_x, const Mat& grad_y, double Th1, double Th2, Mat& edges)
{
	//Canny edge 
	Canny(grad_x, grad_y, edges, Th1, Th2, false);

	//Canny Orientation
	uchar* edgesPtr = edges.data;
	short* grad_xPtr = (short*)(grad_x.data);
	short* grad_yPtr = (short*)(grad_y.data);
	for (int i = 0; i<edges.rows*edges.cols; i++)
	{
		if (edgesPtr[i])
			edgesPtr[i] = CANNY_ORIENTATION(grad_xPtr[i], grad_yPtr[i]);
	}

	return;
}

void MyCanny::cannyGray(const Mat& grayImg, Mat& edges)
{
	//CV_Assert(grayImg.channels() == 1);

	//Sobel Gradient, 梯度图像的深度，设定为 CV_16S 避免外溢。 
	Mat grad_x, grad_y;
	Sobel(grayImg, grad_x, CV_16S, 1, 0, 3, 1, 0, BORDER_REPLICATE);
	Sobel(grayImg, grad_y, CV_16S, 0, 1, 3, 1, 0, BORDER_REPLICATE);

	canny(grad_x, grad_y, th1, th2, edges);
	return;
}

void MyCanny::cannyBGR(const Mat& bgrImg, Mat& edges)
{
	if (bgrImg.channels() != 3)
	{
		if (bgrImg.channels() == 1)
		{
			cannyGray(bgrImg, edges);
			return;
		}

		return;
	}

	//CV_Assert(bgrImg.channels() == 3);
	Mat bgrChannels[3];
	split(bgrImg, bgrChannels);

	//Sobel Gradient, 梯度图像的深度，设定为 CV_16S 避免外溢。 
	Mat rgbGrad_x, rgbGrad_y;
	Sobel(bgrChannels[0], rgbGrad_x, CV_16S, 1, 0, 3, 1, 0, BORDER_REPLICATE);
	Sobel(bgrChannels[0], rgbGrad_y, CV_16S, 0, 1, 3, 1, 0, BORDER_REPLICATE);

	Mat grad_x, grad_y;
	Sobel(bgrChannels[1], grad_x, CV_16S, 1, 0, 3, 1, 0, BORDER_REPLICATE);
	Sobel(bgrChannels[1], grad_y, CV_16S, 0, 1, 3, 1, 0, BORDER_REPLICATE);
	rgbGrad_x += grad_x;
	rgbGrad_y += grad_y;

	Sobel(bgrChannels[2], grad_x, CV_16S, 1, 0, 3, 1, 0, BORDER_REPLICATE);
	Sobel(bgrChannels[2], grad_y, CV_16S, 0, 1, 3, 1, 0, BORDER_REPLICATE);
	rgbGrad_x += grad_x;
	rgbGrad_y += grad_y;

	canny(rgbGrad_x, rgbGrad_y, 3.0*th1, 3.0*th2, edges);
}

struct greaterThanPtr: public std::binary_function<const float *, const float *, bool>
{
	bool operator () (const float * a, const float * b) const
		// Ensure a fully deterministic result of the sort
	{
		return (*a > *b) ? true : (*a < *b) ? false : (a > b);
	}
};

int MyCanny::featuresLocalMaxEigen(const Mat& image, Mat& eigenVal, Mat& localMax, double qualityLevel, InputArray _mask, int blockSize, bool useHarrisDetector, double harrisK)
{
	if (image.empty())
	{
		if (!localMax.empty())
			localMax.setTo(0);
		return 0;
	}

	int rows = image.rows;
	int cols = image.cols;

	Mat mask = _mask.getMat();

	//Eigen Value
	if (useHarrisDetector)
		cornerHarris(image, eigenVal, blockSize, 3, harrisK);
	else
		cornerMinEigenVal(image, eigenVal, blockSize, 3);

	//threshloding
	double maxVal = 0;
	minMaxLoc(eigenVal, 0, &maxVal, 0, 0, _mask);
	threshold(eigenVal, eigenVal, maxVal*qualityLevel, 0, THRESH_TOZERO);

	//dilate
	Mat tmp;
	dilate(eigenVal, tmp, Mat());

	// collect list of pointers to features - put them into temporary image
	localMax.create(rows, cols, CV_8UC1);
	localMax.setTo(0);

	int num = 0;
	float* eig_data = (float*)(eigenVal.data);
	float* tmp_data = (float*)(tmp.data);
	uchar* max_data = (uchar*)(localMax.data);
	uchar* mask_data = (uchar*)(mask.data);
	bool noMask = mask_data ? false : true;
	for (int y = 1; y < rows - 1; y++)
	{
		eig_data += cols;
		tmp_data += cols;
		max_data += cols;
		mask_data += cols;

		for (int x = 1; x < cols - 1; x++)
		{
			float val = eig_data[x];
			if (val > 0 && val == tmp_data[x] && (noMask || mask_data[x]))
			{
				max_data[x] = 1;
				num++;
			}
		}
	}

	return num;
}

void MyCanny::myGoodFeaturesToTrack(const Mat& eigenVal, const Mat& localMax, vector<Point>& corners, int maxCorners, double minDistance, InputArray _mask)
{
	corners.clear();
	if (localMax.empty())
		return;

	int rows = localMax.rows;
	int cols = localMax.cols;

	Mat mask = _mask.getMat();

	// collect list of pointers to features - put them into temporary image
	vector<const float*> tmpCorners;

	float* eig_data = (float*)(eigenVal.data);
	uchar* max_data = (uchar*)(localMax.data);
	uchar* mask_data = (uchar*)(mask.data);
	bool noMask = mask_data ? false : true;
	for (int y = 1; y < rows - 1; y++)
	{
		eig_data += cols;
		max_data += cols;
		mask_data += cols;

		for (int x = 1; x < cols - 1; x++)
		{
			if (max_data[x] && (noMask || mask_data[x]))
				tmpCorners.push_back(eig_data + x);
		}
	}

	//////////////////////////////////////////////////////////////////////////////////////////////
	size_t total = tmpCorners.size(), ncorners = 0;
	if (total == 0)
		return;

	sort(tmpCorners.begin(), tmpCorners.end(), greaterThanPtr());

	if (minDistance >= 1)
	{
		// Partition the image into larger grids
		int w = localMax.cols;
		int h = localMax.rows;

		const int cell_size = cvRound(minDistance);
		const int grid_width = (w + cell_size - 1) / cell_size;
		const int grid_height = (h + cell_size - 1) / cell_size;

		std::vector<std::vector<Point2f> > grid(grid_width*grid_height);

		minDistance *= minDistance;

		for (int i = 0; i < total; i++)
		{
			int ofs = (int)((const uchar*)tmpCorners[i] - eigenVal.ptr());
			int y = (int)(ofs / eigenVal.step);
			int x = (int)((ofs - y*eigenVal.step) / sizeof(float));

			bool good = true;

			int x_cell = x / cell_size;
			int y_cell = y / cell_size;

			int x1 = x_cell - 1;
			int y1 = y_cell - 1;
			int x2 = x_cell + 1;
			int y2 = y_cell + 1;

			// boundary check
			x1 = std::max(0, x1);
			y1 = std::max(0, y1);
			x2 = std::min(grid_width - 1, x2);
			y2 = std::min(grid_height - 1, y2);

			for (int yy = y1; yy <= y2; yy++)
			{
				for (int xx = x1; xx <= x2; xx++)
				{
					std::vector <Point2f> &m = grid[yy*grid_width + xx];

					if (m.size())
					{
						for (int j = 0; j < m.size(); j++)
						{
							float dx = x - m[j].x;
							float dy = y - m[j].y;

							if (dx*dx + dy*dy < minDistance)
							{
								good = false;
								goto break_out;
							}
						}
					}
				}
			}

		break_out:

			if (good)
			{
				grid[y_cell*grid_width + x_cell].push_back(Point2f((float)x, (float)y));

				corners.push_back(Point(x, y));
				++ncorners;

				if (maxCorners > 0 && (int)ncorners == maxCorners)
					break;
			}
		}
	}
	else
	{
		for (int i = 0; i < total; i++)
		{
			int ofs = (int)((const uchar*)tmpCorners[i] - eigenVal.ptr());
			int y = (int)(ofs / eigenVal.step);
			int x = (int)((ofs - y*eigenVal.step) / sizeof(float));

			corners.push_back(Point(x, y));
			++ncorners;
			if (maxCorners > 0 && (int)ncorners == maxCorners)
				break;
		}
	}

	return;
}

bool MyCanny::cornerDescriptorExtractor(Mat& greyImg, int bufferSize, DescriptorNCCHead* descriptor)
{
	int size = descriptor->size;
	descriptor->size = 0;
	if (size < 2 || bufferSize < size * size)
		return false;

	int rows = greyImg.rows;
	int cols = greyImg.cols;

	int y = descriptor->corner.y;
	int x = descriptor->corner.x;

	//check border
	int y0 = y - size / 2;
	if (y0 < 0 || y0 >= rows)
		return false;
	int x0 = x - size / 2;
	if (x0 < 0 || x0 >= cols)
		return false;

	int y1 = y0 + size - 1;
	if (y1 < 0 || y1 >= rows)
		return false;
	int x1 = x0 + size - 1;
	if (x1 < 0 || x1 >= cols)
		return false;

	//get descriptor
	uchar* desPtr = (uchar*)&(descriptor->descriptor);
	uchar* imgPtr = (uchar*)(greyImg.data) + y0 * cols + x0;
	for (int i = 0; i < size; i++)
	{
		memcpy(desPtr, imgPtr, size);
		desPtr += size;
		imgPtr += cols;
	}

	descriptor->size = size;
	return true;
}

bool MyCanny::cornerDescriptorExtractor(Mat& greyImg, int size, int devision, int bufferSize, DescriptorPAIRHead* descriptor)
{
	ushort& len = descriptor->len;
	len = 0;
	if (size < 2 || devision > (size - 2))
		return false;

	int rows = greyImg.rows;
	int cols = greyImg.cols;

	int y = descriptor->corner.y;
	int x = descriptor->corner.x;

	//check border
	int y0 = y - size / 2;
	if (y0 < 0 || y0 >= rows)
		return false;
	int x0 = x - size / 2;
	if (x0 < 0 || x0 >= cols)
		return false;

	int y1 = y0 + size - 1;
	if (y1 < 0 || y1 >= rows)
		return false;
	int x1 = x0 + size - 1;
	if (x1 < 0 || x1 >= cols)
		return false;

	//steps
	vector<int> steps(devision);
	double dd0 = (double)(size - 1) / double(devision + 1);
	for (int i = 1; i <= devision; i++)
		steps[i - 1] = int(i * dd0 + 0.5);

	//data
	vector<uchar> data(4 * (devision + 1));

	int num = 0;
	uchar* ptr0 = (uchar*)(greyImg.data) + y0 * cols;
	uchar* ptr1 = (uchar*)(greyImg.data) + y1 * cols;

	data[num++] = ptr0[x0]; data[num++] = ptr0[x1];
	data[num++] = ptr1[x0]; data[num++] = ptr1[x1];
	for (int i = 0; i < devision; i++)
	{
		int k = x0 + steps[i];

		data[num++] = ptr0[k];
		data[num++] = ptr1[k];
	}

	for (int i = 0; i < devision; i++)
	{
		uchar* ptr = ptr0 + steps[i] * cols;

		data[num++] = ptr[x0];
		data[num++] = ptr[x1];
	}

	//get descriptor
	uchar* desPtr = (uchar*)&(descriptor->descriptor);
	memset(desPtr, 0, bufferSize);

	uchar aa = 0x01; int k = 0;
	for (int i = 0; i < num; i++)
	{
		for (int j = i + 1; j < num; j++)
		{
			//CV_Assert(k < bufferSize);
			if (data[i] > data[j])
				desPtr[k] |= aa;
			len++;

			aa <<= 1; 
			if (!aa)
			{
				aa = 0x01;
				k++;
			}
		}
	}

	//CV_Assert(len == num * (num - 1) / 2);

	return true;
}

bool MyCanny::cornerDescriptorExtractor(Mat& greyImg, DescriptorNCC9& descriptor)
{
	descriptor.size = 9;
	return cornerDescriptorExtractor(greyImg, sizeof(descriptor.descriptor), (DescriptorNCCHead*)(&descriptor));
}

bool MyCanny::cornerDescriptorExtractor(Mat& greyImg, int size, DescriptorPAIR0& descriptor)
{
	return cornerDescriptorExtractor(greyImg, size, 0, sizeof(descriptor.descriptor), (DescriptorPAIRHead*)(&descriptor));
}

bool MyCanny::cornerDescriptorExtractor(Mat& greyImg, int size, DescriptorPAIR1& descriptor)
{
	return cornerDescriptorExtractor(greyImg, size, 1, sizeof(descriptor.descriptor), (DescriptorPAIRHead*)(&descriptor));
}

bool MyCanny::cornerDescriptorExtractor(Mat& greyImg, int size, DescriptorPAIR2& descriptor)
{
	return cornerDescriptorExtractor(greyImg, size, 2, sizeof(descriptor.descriptor), (DescriptorPAIRHead*)(&descriptor));
}

bool MyCanny::cornerDescriptorExtractor(Mat& greyImg, int size, DescriptorPAIR3& descriptor)
{
	return cornerDescriptorExtractor(greyImg, size, 3, sizeof(descriptor.descriptor), (DescriptorPAIRHead*)(&descriptor));
}

double MyCanny::cornerDescriptorDistance(DescriptorNCCHead* descriptor1, DescriptorNCCHead* descriptor2)
{
	uchar size = descriptor1->size;
	if (size != descriptor2->size || size == 0)
		return double(0.0);
	int size2 = int(size) * int(size);

	uchar* desA = (uchar*)&(descriptor1->descriptor);
	uchar* desB = (uchar*)&(descriptor2->descriptor);

	int sumA = 0, sumB = 0, sumAB = 0, sumA2 = 0, sumB2 = 0;
	for (int i = 0; i < size2; i++)
	{
		int vA = *(desA++);
		int vB = *(desB++);

		sumA += vA;
		sumB += vB;
		sumAB += vA * vB;
		sumA2 += vA * vA;
		sumB2 += vB * vB;
	}

	sumAB = sumAB * size2 - sumA * sumB;
	sumA2 = sumA2 * size2 - sumA * sumA;
	sumB2 = sumB2 * size2 - sumB * sumB;

	if (sumA2 == 0 || sumB2 == 0)
	{
		if (sumA2 == sumB2)
			return 1.0;
		return 0.0;
	}

	return double(sumAB) / (sqrt(double(sumA2)) * sqrt(double(sumB2)));
}

double MyCanny::cornerDescriptorDistance(DescriptorPAIRHead* descriptor1, DescriptorPAIRHead* descriptor2)
{
	int len = descriptor1->len;
	if (len != descriptor2->len || len == 0)
		return double(1.0);

	uchar* des1 = (uchar*)&(descriptor1->descriptor);
	uchar* des2 = (uchar*)&(descriptor2->descriptor);

	int bytes = len / 8;
	int distance = 0;
	for (int i = 0; i < bytes; i++)
	{
		uchar bb = des1[i] ^ des2[i];
		for (int j = 0; j < 8; j++)
		{
			distance += bb & 0x01;
			bb >>= 1;
		}
	}

	int bits = len % 8;
	if (!bits)
	{
		uchar bb = des1[bytes] ^ des2[bytes];
		for (int j = 0; j < bits; j++)
		{
			distance += bb & 0x01;
			bb >>= 1;
		}
	}

	return double(distance) / double(len);
}

double MyCanny::cornerDescriptorDistance(DescriptorNCC9& descriptor1, DescriptorNCC9& descriptor2)
{
	return cornerDescriptorDistance((DescriptorNCCHead*)(&descriptor1), (DescriptorNCCHead*)(&descriptor2));
}

double MyCanny::cornerDescriptorDistance(DescriptorPAIR0& descriptor1, DescriptorPAIR0& descriptor2)
{
	return cornerDescriptorDistance((DescriptorPAIRHead*)(&descriptor1), (DescriptorPAIRHead*)(&descriptor2));
}

double MyCanny::cornerDescriptorDistance(DescriptorPAIR1& descriptor1, DescriptorPAIR1& descriptor2)
{
	return cornerDescriptorDistance((DescriptorPAIRHead*)(&descriptor1), (DescriptorPAIRHead*)(&descriptor2));
}

double MyCanny::cornerDescriptorDistance(DescriptorPAIR2& descriptor1, DescriptorPAIR2& descriptor2)
{
	return cornerDescriptorDistance((DescriptorPAIRHead*)(&descriptor1), (DescriptorPAIRHead*)(&descriptor2));
}

double MyCanny::cornerDescriptorDistance(DescriptorPAIR3& descriptor1, DescriptorPAIR3& descriptor2)
{
	return cornerDescriptorDistance((DescriptorPAIRHead*)(&descriptor1), (DescriptorPAIRHead*)(&descriptor2));
}
