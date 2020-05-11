#include "stdafx.h"
#include "bodyFilter.h"

//id: -1, 0, 1
//        X
#define maxDxLeft(a, b)	((a) > (b) ?  0 : 1)
#define maxDxRight(a, b)	((a) > (b) ? -1 : 0)
#define maxDx(a, b, c)	((a) > (c) ? (maxDxRight((a), (b))) : (maxDxLeft(b, c)))

#define RemoveEdge(edge) ((edge).y1 = (edge).y0, (edge).x1 = (edge).x0)
#define IsEdgeNotRemoved(edge) ((edge).y1 > (edge).y0 || (edge).x1 > (edge).x0)
#define IsEdgeRemoved(edge) ((edge).y1 == (edge).y0 && (edge).x1 == (edge).x0)

bodyFilter::bodyFilter()
{
}


bodyFilter::~bodyFilter()
{
}

#define STACK_NUM_COPY(num, s_num) (memcpy((num), (s_num), rows * sizeof(ushort)))
#define STACK_PUSH(s, s_head, s_num, x)	((s)[(*s_head)] = (x), (*s_head) = ((*s_head) + 1) % cols, (*s_num)++)
#define STACK_POP(s,  s_rear, s_num, x)	((x) = (s)[(*s_rear)], (*s_rear) = ((*s_rear) + 1) % cols, (*s_num)--)
void bodyFilter::dilateEdge(const Mat& edges, const Mat& seed, Mat& mask, int size)
{
	uchar *edgePtr,  *maskPtr, *seedPtr;
	uchar *edgePtr0, *maskPtr0;
	uchar *edgePtr1, *maskPtr1;

	ushort *stackPtr,  *stackHeadPtr,  *stackRearPtr,  *stackNumPtr;
	ushort *stackPtr0, *stackHeadPtr0, *stackRearPtr0, *stackNumPtr0;
	ushort *stackPtr1, *stackHeadPtr1, *stackRearPtr1, *stackNumPtr1;

	int rows = edges.rows;
	int cols = edges.cols;

	// stack
	Mat stackCtr(4, rows, CV_16UC1);
	memset(stackCtr.data, 0, (4 * rows) * sizeof(ushort));
	ushort* stackHead = (ushort*)(stackCtr.data);
	ushort* stackRear = stackHead + rows;
	ushort* stackNum = stackRear + rows;
	ushort* pointNum = stackNum + rows;

	Mat stackData(rows, cols, CV_16UC1);
	ushort* stack = (ushort*)stackData.data;

	//init
	if (mask.rows != rows || mask.cols != cols)
		mask.create(rows, cols, CV_8U);
	memcpy(mask.data, seed.data, rows * cols);

	seedPtr = seed.data;  stackPtr = stack; stackHeadPtr = stackHead; stackNumPtr = stackNum;
	for (ushort y = 0; y < ushort(rows); y++)
	{
		for (ushort x = 0; x < ushort(cols); x++)
		{
			if (!seedPtr[x])
				continue;

			STACK_PUSH(stackPtr, stackHeadPtr, stackNumPtr, x);
		}

		seedPtr += cols; stackPtr += cols; stackHeadPtr++; stackNumPtr++;
	}

	//dilate
	for (int i = 0; i < size; i++)
	{
		STACK_NUM_COPY(pointNum, stackNum);

		edgePtr  = edges.data;     maskPtr  = mask.data;      stackPtr  = stack;           stackHeadPtr  = stackHead;        stackRearPtr = stackRear;         stackNumPtr  = stackNum;
		edgePtr0 = edgePtr - cols; maskPtr0 = maskPtr - cols; stackPtr0 = stackPtr - cols; stackHeadPtr0 = stackHeadPtr - 1; stackRearPtr0 = stackRearPtr - 1; stackNumPtr0 = stackNumPtr - 1;
		edgePtr1 = edgePtr + cols; maskPtr1 = maskPtr + cols; stackPtr1 = stackPtr + cols; stackHeadPtr1 = stackHeadPtr + 1; stackRearPtr1 = stackRearPtr + 1; stackNumPtr1 = stackNumPtr + 1;
		for (ushort y = 0; y < ushort(rows); y++)
		{
			for (ushort j = 0; j < pointNum[y]; j++)
			{
				ushort x0;
				STACK_POP(stackPtr, stackRearPtr, stackNumPtr, x0);

				//////up//////
				if (y > 0)
				{
					//left
					if (x0 > 0)
					{
						ushort xx0 = x0 - 1;
						if (edgePtr0[xx0] && !maskPtr0[xx0])
						{
							maskPtr0[xx0] = 1; STACK_PUSH(stackPtr0, stackHeadPtr0, stackNumPtr0, xx0);
						}
					}

					//middle
					if (edgePtr0[x0] && !maskPtr0[x0])
					{
						maskPtr0[x0] = 1; STACK_PUSH(stackPtr0, stackHeadPtr0, stackNumPtr0, x0);
					}

					//right
					if (x0 < cols - 1)
					{
						ushort xx0 = x0 + 1;
						if (edgePtr0[xx0] && !maskPtr0[xx0])
						{
							maskPtr0[xx0] = 1; STACK_PUSH(stackPtr0, stackHeadPtr0, stackNumPtr0, xx0);
						}
					}
				}

				//////middle//////
				//left
				if (x0 > 0)
				{
					ushort xx0 = x0 - 1;
					if (edgePtr[xx0] && !maskPtr[xx0])
					{
						maskPtr[xx0] = 1; STACK_PUSH(stackPtr, stackHeadPtr, stackNumPtr, xx0);
					}
				}

				//right
				if (x0 < cols - 1)
				{
					ushort xx0 = x0 + 1;
					if (edgePtr[xx0] && !maskPtr[xx0])
					{
						maskPtr[xx0] = 1; STACK_PUSH(stackPtr, stackHeadPtr, stackNumPtr, xx0);
					}
				}

				//////down//////
				if (y < (rows - 1))
				{
					//left
					if (x0 > 0)
					{
						ushort xx0 = x0 - 1;
						if (edgePtr1[xx0] && !maskPtr1[xx0])
						{
							maskPtr1[xx0] = 1; STACK_PUSH(stackPtr1, stackHeadPtr1, stackNumPtr1, xx0);
						}
					}

					//middle
					if (edgePtr1[x0] && !maskPtr1[x0])
					{
						maskPtr1[x0] = 1; STACK_PUSH(stackPtr1, stackHeadPtr1, stackNumPtr1, x0);
					}

					//right
					if (x0 < cols - 1)
					{
						ushort xx0 = x0 + 1;
						if (edgePtr1[xx0] && !maskPtr1[xx0])
						{
							maskPtr1[xx0] = 1; STACK_PUSH(stackPtr1, stackHeadPtr1, stackNumPtr1, xx0);
						}
					}
				}
			}

			edgePtr  += cols; maskPtr  += cols; stackPtr  += cols; stackHeadPtr++;  stackRearPtr++;  stackNumPtr++;
			edgePtr0 += cols; maskPtr0 += cols; stackPtr0 += cols; stackHeadPtr0++; stackRearPtr0++; stackNumPtr0++;
			edgePtr1 += cols; maskPtr1 += cols; stackPtr1 += cols; stackHeadPtr1++; stackRearPtr1++; stackNumPtr1++;
		}
	}

	return;
}

void bodyFilter::topDownScan(const int rows, const int cols, uchar* mask[], uchar* link[])
{
	char dx;
	uchar len0;

	uchar* maskPtr0 = mask[0];
	for (int j = 0; j < cols; j++)
	{
		if (maskPtr0[j])
			maskPtr0[j] = 1;
	}

	for (int i = 1; i < rows; i++)
	{
		uchar* maskPtr = mask[i];
		uchar* parentPtr = link[i];

		//CV_Assert(cols >= 2);
		if (maskPtr[0])
		{
			dx = maxDxLeft(maskPtr0[0], maskPtr0[1]);
			len0 = maskPtr0[dx];
			if (len0)
			{
				setParent(parentPtr[0], -1, dx); //Dy=-1; Dx=dx;
				maskPtr[0] = 1 + len0;

			}
			else
				maskPtr[0] = 1;
		}

		if (maskPtr[cols - 1])
		{
			dx = maxDxRight(maskPtr0[cols - 2], maskPtr0[cols - 1]);
			len0 = maskPtr0[dx + cols - 1];
			if (len0)
			{
				setParent(parentPtr[cols - 1], -1, dx); //Dy=-1; Dx=dx;
				maskPtr[cols - 1] = 1 + len0;
			}
			else
				maskPtr[cols - 1] = 1;
		}

		for (int j = 1; j < cols - 1; j++)
		{
			if (maskPtr[j])
			{
				dx = maxDx(maskPtr0[j - 1], maskPtr0[j], maskPtr0[j + 1]);
				len0 = maskPtr0[dx + j];
				if (len0)
				{
					setParent(parentPtr[j], -1, dx); //Dy=-1; Dx=dx;
					maskPtr[j] = 1 + len0;
				}
				else
					maskPtr[j] = 1;
			}
		}

		maskPtr0 = maskPtr;
	}

	return;
}

void bodyFilter::leftRightScan(const int rows, const int cols, uchar* mask[], uchar* link[])
{
	char dy;
	uchar len0;

	for (int j = 0; j < rows; j++)
	{
		if (mask[j][0])
			mask[j][0] = 1;
	}

	for (int i = 1; i < cols; i++)
	{
		int i0 = i - 1;

		//CV_Assert(rows >= 2);
		if (mask[0][i])
		{
			dy = maxDxLeft(mask[0][i0], mask[1][i0]);
			len0 = mask[dy][i0];
			if (len0)
			{
				setParent(link[0][i], dy, -1); //Dy=dy; Dx=-1;
				mask[0][i] = 1 + len0;

			}
			else
				mask[0][i] = 1;
		}

		if (mask[rows - 1][i])
		{
			dy = maxDxRight(mask[rows - 2][i0], mask[rows - 1][i0]);
			len0 = mask[dy + rows - 1][i0];
			if (len0)
			{
				setParent(link[rows - 1][i], dy, -1); //Dy=dy; Dx=-1;
				mask[rows - 1][i] = 1 + len0;
			}
			else
				mask[rows - 1][i] = 1;
		}

		for (int j = 1; j < rows - 1; j++)
		{
			if (mask[j][i])
			{
				dy = maxDx(mask[j - 1][i0], mask[j][i0], mask[j + 1][i0]);
				len0 = mask[dy + j][i0];
				if (len0)
				{
					setParent(link[j][i], dy, -1); //Dy=dy; Dx=-1;
					mask[j][i] = 1 + len0;
				}
				else
					mask[j][i] = 1;
			}
		}
	}

	return;
}

void bodyFilter::bottomUpScan(const int rows, const int cols, uchar* mask[], uchar* link[], int len)
{
	char dx, dy, dx0, dy0;
	uchar len0;

	uchar* linkPtr0 = link[rows - 1];
	uchar* maskPtr0 = mask[rows - 1];
	for (int j = 0; j < cols; j++)
	{
		if (maskPtr0[j] < len)
			maskPtr0[j] = 0;
	}

	for (int i = rows - 2; i >= 0; i--)
	{
		uchar* linkPtr = link[i];
		uchar* maskPtr = mask[i];

		// setup link between parent and child
		//CV_Assert(cols >= 2);
		if (maskPtr[0])
		{
			len0 = 0;
			if (maskPtr0[0])
			{
				getParentDYX(linkPtr0[0], dy0, dx0);
				if (dx0 == 0)
				{
					dx = 0;
					len0 = maskPtr0[0];
				}
			}
			if (maskPtr0[1] > len0)
			{
				getParentDYX(linkPtr0[1], dy0, dx0);
				if (dx0 == -1)
				{
					dx = 1;
					len0 = maskPtr0[1];
				}
			}
			if (len0)
				setChild(linkPtr[0], 1, dx); //Dy=1; Dx=dx;
		}

		if (maskPtr[cols - 1])
		{
			len0 = 0;
			if (maskPtr0[cols - 2])
			{
				getParentDYX(linkPtr0[cols - 2], dy0, dx0);
				if (dx0 == 1)
				{
					dx = -1;
					len0 = maskPtr0[cols - 2];
				}
			}
			if (maskPtr0[cols - 1] > len0)
			{
				getParentDYX(linkPtr0[cols - 1], dy0, dx0);
				if (dx0 == 0)
				{
					dx = 0;
					len0 = maskPtr0[cols - 1];
				}
			}
			if (len0)
				setChild(linkPtr[cols - 1], 1, dx); //Dy=1; Dx=dx;
		}

		for (int j = 1; j < cols - 1; j++)
		{
			if (maskPtr[j])
			{
				len0 = 0;
				if (maskPtr0[j - 1])
				{
					getParentDYX(linkPtr0[j - 1], dy0, dx0);
					if (dx0 == 1)
					{
						dx = -1;
						len0 = maskPtr0[j - 1];
					}
				}
				if (maskPtr0[j] > len0)
				{
					getParentDYX(linkPtr0[j], dy0, dx0);
					if (dx0 == 0)
					{
						dx = 0;
						len0 = maskPtr0[j];
					}
				}
				if (maskPtr0[j + 1] > len0)
				{
					getParentDYX(linkPtr0[j + 1], dy0, dx0);
					if (dx0 == -1)
					{
						dx = 1;
						len0 = maskPtr0[j + 1];
					}
				}
				if (len0)
					setChild(linkPtr[j], 1, dx); //Dy=1; Dx=dx;
			}
		}

		// setup new edge by clearing parents
		for (int j = 0; j < cols; j++)
		{
			if (Parent(linkPtr0[j]))
			{
				getParentDYX(linkPtr0[j], dy0, dx0);
				getChildDYX(linkPtr[dx0 + j], dy, dx);
				if (-dx != dx0) // not a pair of parent and child?
				{
					len0 = maskPtr0[j] - maskPtr[dx0 + j];
					if (len0 >= len)
						clearParent(linkPtr0[j]);
				}
			}
		}

		//update length
		for (int j = 0; j < cols; j++)
		{
			if (Child(linkPtr[j]))
			{
				getChildDYX(linkPtr[j], dy, dx);
				maskPtr[j] = maskPtr0[dx + j];
			}
			else
			{
				if (maskPtr[j] < len)
					maskPtr[j] = 0;
			}
		}

		linkPtr0 = linkPtr;
		maskPtr0 = maskPtr;
	}

	return;
}

void bodyFilter::rightLeftScan(const int rows, const int cols, uchar* mask[], uchar* link[], int len)
{
	char dy, dx, dy0, dx0;
	uchar len0;

	for (int j = 0; j < rows; j++)
	{
		if (mask[j][cols - 1] < len)
			mask[j][cols - 1] = 0;
	}

	for (int i = cols - 2; i >= 0; i--)
	{
		int i0 = i + 1;

		// setup link between parent and child
		//CV_Assert(rows >= 2);
		if (mask[0][i])
		{
			len0 = 0;
			if (mask[0][i0])
			{
				getParentDYX(link[0][i0], dy0, dx0);
				if (dy0 == 0)
				{
					dy = 0;
					len0 = mask[0][i0];
				}
			}
			if (mask[1][i0] > len0)
			{
				getParentDYX(link[1][i0], dy0, dx0);
				if (dy0 == -1)
				{
					dy = 1;
					len0 = mask[1][i0];
				}
			}
			if (len0)
				setChild(link[0][i], dy, 1); //Dy=dy; Dx=1;
		}

		if (mask[rows - 1][i])
		{
			len0 = 0;
			if (mask[rows - 2][i0])
			{
				getParentDYX(link[rows - 2][i0], dy0, dx0);
				if (dy0 == 1)
				{
					dy = -1;
					len0 = mask[rows - 2][i0];
				}
			}
			if (mask[rows - 1][i0] > len0)
			{
				getParentDYX(link[rows - 1][i0], dy0, dx0);
				if (dy0 == 0)
				{
					dy = 0;
					len0 = mask[rows - 1][i0];
				}
			}
			if (len0)
				setChild(link[rows - 1][i], dy, 1); //Dy=dy; Dx=1;
		}

		for (int j = 1; j < rows - 1; j++)
		{
			if (mask[j][i])
			{
				len0 = 0;
				if (mask[j - 1][i0])
				{
					getParentDYX(link[j - 1][i0], dy0, dx0);
					if (dy0 == 1)
					{
						dy = -1;
						len0 = mask[j - 1][i0];
					}
				}
				if (mask[j][i0] > len0)
				{
					getParentDYX(link[j][i0], dy0, dx0);
					if (dy0 == 0)
					{
						dy = 0;
						len0 = mask[j][i0];
					}
				}
				if (mask[j + 1][i0] > len0)
				{
					getParentDYX(link[j + 1][i0], dy0, dx0);
					if (dy0 == -1)
					{
						dy = 1;
						len0 = mask[j + 1][i0];
					}
				}
				if (len0)
					setChild(link[j][i], dy, 1); //Dy=dy; Dx=1;
			}
		}

		// setup new edge by clearing parents
		for (int j = 0; j < rows; j++)
		{
			if (Parent(link[j][i0]))
			{
				getParentDYX(link[j][i0], dy0, dx0);
				getChildDYX(link[dy0 + j][i], dy, dx);
				if (-dy != dy0) // not a pair of parent and child?
				{
					len0 = mask[j][i0] - mask[dy0 + j][i];
					if (len0 >= len)
						clearParent(link[j][i0]);
				}
			}
		}
		
		//update length
		for (int j = 0; j < rows; j++)
		{
			if (Child(link[j][i]))
			{
				getChildDYX(link[j][i], dy, dx);
				mask[j][i] = mask[dy + j][i0];
			}
			else
			{
				if (mask[j][i] < len)
					mask[j][i] = 0;
			}
		}
	}

	return;
}

void bodyFilter::vEdges(Mat& mask, Mat& link, int len)
{
	//CV_Assert(mask.depth() == CV_8U); // 8位无符号
	//CV_Assert(mask.rows >= 2 && mask.cols >= 2);
	int rows = mask.rows;
	int cols = mask.cols;

	if (link.rows != rows || link.cols != cols)
		link.create(rows, cols, CV_8U);
	memset(link.data, 0x00, rows * cols);

	//vertical scan
	uchar* maskPtr[MAX_IMAGE_HEIGHT]; uchar* maskPtr0 = mask.data;
	uchar* linkPtr[MAX_IMAGE_HEIGHT]; uchar* linkPtr0 = link.data;
	int i;
	for (i = 0; i < rows && i < MAX_IMAGE_HEIGHT; i++)
	{
		maskPtr[i] = maskPtr0; linkPtr[i] = linkPtr0;
		maskPtr0 += cols; linkPtr0 += cols;
	}

	topDownScan(i, cols, maskPtr, linkPtr);
	bottomUpScan(i, cols, maskPtr, linkPtr, len);
	return;
}

void bodyFilter::hEdges(Mat& mask, Mat& link, int len)
{
	//CV_Assert(mask.depth() == CV_8U); // 8位无符号
	//CV_Assert(mask.rows >= 2 && mask.cols >= 2);
	int rows = mask.rows;
	int cols = mask.cols;

	if (link.rows != rows || link.cols != cols)
		link.create(rows, cols, CV_8U);
	memset(link.data, 0x00, rows * cols);

	//horizontal scan
	uchar* maskPtr[MAX_IMAGE_HEIGHT]; uchar* maskPtr0 = mask.data; 
	uchar* linkPtr[MAX_IMAGE_HEIGHT]; uchar* linkPtr0 = link.data;
	int i;
	for (i = 0; i < rows && i < MAX_IMAGE_HEIGHT; i++)
	{
		maskPtr[i] = maskPtr0; linkPtr[i] = linkPtr0;
		maskPtr0 += cols; linkPtr0 += cols;
	}

	leftRightScan(i, cols, maskPtr, linkPtr);
	rightLeftScan(i, cols, maskPtr, linkPtr, len);

	return;
}

void bodyFilter::getEdgeMask(const Mat& mask, const Mat& link, Mat& bodyEdgeMask)
{
	int rows = mask.rows;
	int cols = mask.cols;
	size_t len = rows * cols;

	// init
	if (bodyEdgeMask.rows != rows || bodyEdgeMask.cols != cols)
		bodyEdgeMask.create(rows, cols, CV_8U);
	memset(bodyEdgeMask.data, 0, rows * cols);

	// get edge map
	uchar* maskPtr = mask.data;
	uchar* linkPtr = link.data;
	uchar* edgeMaskPtr = (uchar*)(bodyEdgeMask.data);

	for (ushort y0 = 0; y0 < rows; y0++)
	{
		for (ushort x0 = 0; x0 < cols; x0++)
		{
			if (isRoot(maskPtr[x0], linkPtr[x0]))
			{
				edgeMaskPtr[x0] = 1;

				int y = y0; int x = x0;
				uchar* linkPtr0 = linkPtr; uchar* edgeMaskPtr0 = edgeMaskPtr;
				while (1)
				{
					uchar child = Child(linkPtr0[x]);
					if (!child)
						break;

					int dy, dx;
					getChildDYX(child, dy, dx);

					y += dy; x += dx;
					//CV_Assert((y >= 0 && y < rows) && (x >= 0 && x < cols));

					if (dy == -1)
					{
						linkPtr0 -= cols; edgeMaskPtr0 -= cols;
					}
					else
					{
						if (dy) //dy==1
						{
							linkPtr0 += cols; edgeMaskPtr0 += cols;
						}
					}

					edgeMaskPtr0[x] = 1;
				}
			}
		}

		maskPtr += cols; linkPtr += cols; edgeMaskPtr += cols;
	}

	return;
}

void bodyFilter::getEdgeMap(const Mat& greyImg, const Mat& edges, const Mat& mask, const Mat& link, EDGEMAP& edgeMap)
{
	int rows = greyImg.rows;
	int cols = greyImg.cols;
	size_t len = rows * cols;

	// init
	if (edgeMap.greyImg.rows != rows || edgeMap.greyImg.cols != cols)
		edgeMap.greyImg.create(rows, cols, CV_8U);
	memcpy(edgeMap.greyImg.data, greyImg.data, len);

	if (edgeMap.link.rows != rows || edgeMap.link.cols != cols)
		edgeMap.link.create(rows, cols, CV_8U);
	memcpy(edgeMap.link.data, link.data, len);

	Mat& map = edgeMap.map;
	if (map.rows != rows || map.cols != cols)
		map.create(rows, cols, CV_16UC1);
	memset(map.data, 0, 2 * rows * cols);

	// get edge map
	EDGE *edge = edgeMap.edge;

	uchar* maskPtr = mask.data;
	uchar* linkPtr = link.data;
	ushort* mapPtr = (ushort*)(map.data);

	ushort num = 0;
	for (ushort y0 = 0; y0 < rows; y0++)
	{
		for (ushort x0 = 0; x0 < cols; x0++)
		{
			if (isRoot(maskPtr[x0], linkPtr[x0]))
			{
				if (num >= MAX_EDGE_NUM)
					break;

				EDGE* edge0 = edge + num;
				//memset(edge0, 0, sizeof(EDGE)); //clear edge

				num++;
				mapPtr[x0] = num;

				edge0->y0 = y0; edge0->x0 = x0;

				int minY = y0; int maxY = y0;
				int minX = x0; int maxX = x0;

				int y = y0; int x = x0;
				uchar* linkPtr0 = linkPtr; ushort* mapPtr0 = mapPtr;
				while (1)
				{
					uchar child = Child(linkPtr0[x]);
					if (!child)
						break;

					int dy, dx;
					getChildDYX(child, dy, dx);

					y += dy; x += dx;
					//CV_Assert((y >= 0 && y < rows) && (x >= 0 && x < cols));

					if (minY > y)
						minY = y;
					if (maxY < y)
						maxY = y;

					if (minX > x)
						minX = x;
					if (maxX < x)
						maxX = x;

					if (dy == -1)
					{
						linkPtr0 -= cols; mapPtr0 -= cols;
					}
					else
					{
						if (dy) //dy==1
						{
							linkPtr0 += cols; mapPtr0 += cols;
						}
					}

					mapPtr0[x] = num;
				}

				edge0->y1 = ushort(y); edge0->x1 = ushort(x);
				edge0->minY = ushort(minY); edge0->maxY = ushort(maxY);
				edge0->minX = ushort(minX); edge0->maxX = ushort(maxX);
			}
		}

		maskPtr += cols;
		linkPtr += cols;
		mapPtr += cols;
	}

	edgeMap.edgeNum = num;
	return;
}

int bodyFilter::showEdgeMap(const String name, EDGEMAP& edgeMap, bool save)
{
	ushort edgesNum = edgeMap.edgeNum;
	EDGE* edges = edgeMap.edge;	
	EdgeTrackingInfo* trackingInfo = edgeMap.trackingInfo;
	uchar* bodyEdgeTag = edgeMap.bodyEdgeTag;

	Mat image;
	cvtColor(edgeMap.greyImg, image, CV_GRAY2BGR);
	Mat& link = edgeMap.link;
	Mat& map = edgeMap.map;

	int rows = link.rows;
	int cols = link.cols;

	uchar edgeShown[MAX_EDGE_NUM];
	memset(edgeShown, 0, MAX_EDGE_NUM);

	int shownEdgeNum = 0;

	uchar* imagePtr = image.data;
	uchar* linkPtr = link.data;
	ushort* mapPtr = (ushort*)(map.data);
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
				shownEdgeNum++;

				imagePtr[3 * x0] = 0;
				imagePtr[3 * x0 + 1] = 0;
				imagePtr[3 * x0 + 2] = 255;

				int y = y0; int x = x0;
				uchar* imagePtr0 = imagePtr; uchar* linkPtr0 = linkPtr; ushort* mapPtr0 = mapPtr;
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
						imagePtr0 -= 3 * cols; linkPtr0 -= cols; mapPtr0 -= cols;
					}
					else
					{
						if (dy) //dy==1
						{
							imagePtr0 += 3 * cols; linkPtr0 += cols; mapPtr0 += cols;
						}
					}

					uchar parent = Parent(linkPtr0[x]);
					CV_Assert(parent);
					int dy0, dx0;
					getParentDYX(parent, dy0, dx0);
					CV_Assert(dy0 == -dy && dx0 == -dx);

					CV_Assert(mapPtr0[x] == edgeId);
					if (isLeaf(mapPtr0[x], linkPtr0[x]))
					{
						CV_Assert(edge.y1 == y && edge.x1 == x);
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

		imagePtr += 3 * cols;
		linkPtr += cols;
		mapPtr += cols;
	}

	for (int i = 0; i < edgesNum; i++)
		CV_Assert(edgeShown[i]);
	CV_Assert(shownEdgeNum == edgesNum);

	imshow(name, image);
	if (save)
		imwrite("testImage\\" + name + ".bmp", image);

	return shownEdgeNum;
}

void bodyFilter::smoothEdgeMap(EDGEMAP& edgeMap)
{
	uchar child;
	int dy, dx;
	int y0, y1, y2, x0, x1, x2;
	uchar *linkPtr0, *linkPtr1, *linkPtr2;
	ushort *mapPtr0, *mapPtr1, *mapPtr2;

	int rows = edgeMap.link.rows;
	int cols = edgeMap.link.cols;

	for (ushort edgeId = 1; edgeId <= edgeMap.edgeNum; edgeId++)
	{
		EDGE& edge = edgeMap.edge[edgeId - 1];

		// init y1, x1
		y1 = edge.y0; x1 = edge.x0;
		linkPtr1 = edgeMap.link.ptr<uchar>(y1); mapPtr1 = edgeMap.map.ptr<ushort>(y1);

		// init y2, x2
		child = Child(linkPtr1[x1]);
		if (!child)
			continue;
		getChildDYX(child, dy, dx);
		y2 = y1 + dy; x2 = x1 + dx;
		//CV_Assert((y2 >= 0 && y2 < rows) && (x2 >= 0 && x2 < cols));
		if (dy == -1)
		{
			linkPtr2 = linkPtr1 - cols; mapPtr2 = mapPtr1 - cols;
		}
		else if (dy) //dy==1
		{
			linkPtr2 = linkPtr1 + cols; mapPtr2 = mapPtr1 + cols;
		}
		else
		{
			linkPtr2 = linkPtr1; mapPtr2 = mapPtr1;
		}

		while (1)
		{
			//set y0, x0
			y0 = y1; x0 = x1;
			linkPtr0 = linkPtr1; mapPtr0 = mapPtr1;

			//set y1, x1
			y1 = y2; x1 = x2;
			linkPtr1 = linkPtr2; mapPtr1 = mapPtr2;

			//set y2, x2
			child = Child(linkPtr1[x1]);
			if (!child)
				break;
			getChildDYX(child, dy, dx);
			y2 = y1 + dy; x2 = x1 + dx;
			//CV_Assert((y2 >= 0 && y2 < rows) && (x2 >= 0 && x2 < cols));
			if (dy == -1)
			{
				linkPtr2 = linkPtr1 - cols; mapPtr2 = mapPtr1 - cols;
			}
			else if (dy) //dy==1
			{
				linkPtr2 = linkPtr1 + cols; mapPtr2 = mapPtr1 + cols;
			}
			else
			{
				linkPtr2 = linkPtr1; mapPtr2 = mapPtr1;
			}

			//Horizontal map
			if (y0 == y2 && y0 != y1)
			{
				if (mapPtr0[x1])
					continue;

				//0 
				clearChild(linkPtr0[x0]);
				setChild(linkPtr0[x0], 0, 1);

				//1 
				linkPtr1[x1] = 0; mapPtr1[x1] = 0; //clear

				y1 = y0;
				linkPtr1 = linkPtr0;  mapPtr1 = mapPtr0;

				mapPtr1[x1] = edgeId;
				linkPtr1[x1] = 0; //clear link before set its child and parent
				setChild(linkPtr1[x1], 0, 1); setParent(linkPtr1[x1], 0, -1);

				//2
				clearParent(linkPtr2[x2]);
				setParent(linkPtr2[x2], 0, -1);
			}

			//Vertical map
			if (x0 == x2 && x0 != x1)
			{
				if (mapPtr1[x0])
					continue;

				//0 
				clearChild(linkPtr0[x0]);
				setChild(linkPtr0[x0], 1, 0);

				//1 
				linkPtr1[x1] = 0; mapPtr1[x1] = 0;

				x1 = x0;

				mapPtr1[x1] = edgeId;
				linkPtr1[x1] = 0; //clear link before set its child and parent
				setChild(linkPtr1[x1], 1, 0); setParent(linkPtr1[x1], -1, 0);

				//2
				clearParent(linkPtr2[x2]);
				setParent(linkPtr2[x2], -1, 0);
			}
		}
	}

	return;
}

ushort bodyFilter::splitEdge(EDGEMAP& edgeMap, ushort edgeId, const int y, const int x)
{
	// too many edges?
	if (edgeMap.edgeNum >= MAX_EDGE_NUM)
		return 0;

	//CV_Assert(edgeId > 0 && edgeId <= edgeMap.edgeNum);
	EDGE& edge0 = edgeMap.edge[edgeId - 1];
	int rows = edgeMap.link.rows; int cols = edgeMap.link.cols;

	////split
	uchar* linkPtr0 = edgeMap.link.ptr<uchar>(y);
	uchar parent = Parent(linkPtr0[x]);
	if (!parent)
		return 0;

	uchar* linkPtr1 = linkPtr0;
	ushort* mapPtr1 = edgeMap.map.ptr<ushort>(y);
	if (mapPtr1[x] != edgeId)
		return 0;

	clearParent(linkPtr0[x]);

	//CV_Assert((y >= edge0.minY && y <= edge0.maxY) && (x >= edge0.minX && x <= edge0.maxX));
	int dy, dx;
	getParentDYX(parent, dy, dx);
	int y1 = y + dy; 	int x1 = x + dx;
	//CV_Assert((y1 >= edge0.minY && y1 <= edge0.maxY) && (x1 >= edge0.minX && x1 <= edge0.maxX));

	if (dy == -1)
	{
		linkPtr1 -= cols;
		mapPtr1 -= cols;
	}
	else if (dy == 1)
	{
		linkPtr1 += cols;
		mapPtr1 += cols;
	}
	clearChild(linkPtr1[x1]);

	////edge1
	EDGE& edge1 = edgeMap.edge[edgeMap.edgeNum];
	ushort edgeId1 = edgeMap.edgeNum + 1;
	edgeMap.edgeNum = edgeId1;

	edge1 = edge0;
	edge1.y1 = y1; edge1.x1 = x1;

	int minY = y1; int maxY = y1;
	int minX = x1; int maxX = x1;
	while (1)
	{
		mapPtr1[x1] = edgeId1;

		if (isRoot(mapPtr1[x1], linkPtr1[x1]))
			break;

		getParentDYX(linkPtr1[x1], dy, dx);
		y1 += dy; x1 += dx;
		//CV_Assert((y1 >= edge0.minY && y1 <= edge0.maxY) && (x1 >= edge0.minX && x1 <= edge0.maxX));

		if (minY > y1)
			minY = y1;
		if (maxY < y1)
			maxY = y1;

		if (minX > x1)
			minX = x1;
		if (maxX < x1)
			maxX = x1;

		if (dy == -1)
		{
			linkPtr1 -= cols;
			mapPtr1 -= cols;
		}
		else if (dy == 1)
		{
			linkPtr1 += cols;
			mapPtr1 += cols;
		}
	}

	edge1.minY = ushort(minY); edge1.maxY = ushort(maxY);
	edge1.minX = ushort(minX); edge1.maxX = ushort(maxX);

	////edge0 (a new edge)
	int y0 = y; int x0 = x;
	edge0.y0 = y0; edge0.x0 = x0;

	minY = y0; maxY = y0;
	minX = x0; maxX = x0;
	while (1)
	{
		uchar child = Child(linkPtr0[x0]);
		if (!child)
			break;

		getChildDYX(child, dy, dx);
		y0 += dy; x0 += dx;
		//CV_Assert((y0 >= edge0.minY && y0 <= edge0.maxY) && (x0 >= edge0.minX && x0 <= edge0.maxX));

		if (minY > y0)
			minY = y0;
		if (maxY < y0)
			maxY = y0;

		if (minX > x0)
			minX = x0;
		if (maxX < x0)
			maxX = x0;

		if (dy == -1)
			linkPtr0 -= cols;
		else
		{
			if (dy) //dy==1
				linkPtr0 += cols;
		}
	}

	edge0.minY = ushort(minY); edge0.maxY = ushort(maxY);
	edge0.minX = ushort(minX); edge0.maxX = ushort(maxX);

	return edgeId1;
}

void bodyFilter::mergeEdges(EDGEMAP& edgeMap, ushort edgeId0, ushort edgeId1, int* bridgeDy, int* bridgeDx, int bridegLen)
{
	uchar child;
	int dy, dx;

	int rows = edgeMap.link.rows;
	int cols = edgeMap.link.cols;

	EDGE& edge0 = edgeMap.edge[edgeId0 - 1];
	EDGE& edge1 = edgeMap.edge[edgeId1 - 1];

	//edge0
	int y = edge0.y0; int x = edge0.x0;
	uchar* linkPtr = edgeMap.link.ptr<uchar>(y); ushort* mapPtr = edgeMap.map.ptr<ushort>(y);
	while (1)
	{
		//CV_Assert(mapPtr[x] = edgeId0);
		mapPtr[x] = edgeId1;

		child = Child(linkPtr[x]);
		if (!child)
			break;

		getChildDYX(child, dy, dx);
		y += dy; x += dx;
		//CV_Assert((y >= 0 && y < rows) && (x >= 0 && x < cols));

		if (dy == -1)
		{
			linkPtr -= cols; mapPtr -= cols;
		}
		else
		{
			if (dy) //dy==1
			{
				linkPtr += cols; mapPtr += cols;
			}
		}
	}

	//CV_Assert(y == edge0.y1 && x == edge0.x1);

	ushort minY = edge0.minY; ushort maxY = edge0.maxY;
	ushort minX = edge0.minX; ushort maxX = edge0.maxX;

	//bridge
	for (int i = 0; i < bridegLen; i++)
	{
		dy = bridgeDy[i]; dx = bridgeDx[i];
		//CV_Assert((dy >= -1 && dy <= 1) && (dx >= -1 && dx <= 1) && (dx != 0 || dy != 0));
		clearChild(linkPtr[x]); setChild(linkPtr[x], dy, dx);

		y += dy; x += dx;
		//CV_Assert((y >= 0 && y < rows) && (x >= 0 && x < cols));

		if (minY > y)
			minY = y;
		if (maxY < y)
			maxY = y;

		if (minX > x)
			minX = x;
		if (maxX < x)
			maxX = x;

		if (dy == -1)
		{
			linkPtr -= cols; mapPtr -= cols;
		}
		else
		{
			if (dy) //dy==1
			{
				linkPtr += cols; mapPtr += cols;
			}
		}

		//set map and link
		//CV_Assert((i < (bridegLen - 1) && !mapPtr[x]) || (i == (bridegLen - 1) && mapPtr[x] == edgeId1));
		mapPtr[x] = edgeId1;

		clearParent(linkPtr[x]); setParent(linkPtr[x], -dy, -dx);
	}

	//edge1
	//CV_Assert(y == edge1.y0 && x == edge1.x0);
	edge1.y0 = edge0.y0; edge1.x0 = edge0.x0;
	if (edge1.minY > minY)
		edge1.minY = minY;
	if (edge1.maxY < maxY)
		edge1.maxY = maxY;

	if (edge1.minX > minX)
		edge1.minX = minX;
	if (edge1.maxX < maxX)
		edge1.maxX = maxX;

	//remove edge0
	RemoveEdge(edge0);

	return;
}

void bodyFilter::clearEdge(EDGEMAP& edgeMap, ushort edgeId)
{
	EDGE& edge = edgeMap.edge[edgeId-1];

	int rows = edgeMap.link.rows;
	int cols = edgeMap.link.cols;

	int y = edge.y0; int x = edge.x0;
	ushort* mapPtr = edgeMap.map.ptr<ushort>(y); uchar* linkPtr = edgeMap.link.ptr<uchar>(y);
	while (1)
	{
		uchar child = Child(linkPtr[x]);

		mapPtr[x] = 0;
		linkPtr[x] = 0;

		if (!child)
			break;
		int dy, dx;
		getChildDYX(child, dy, dx);
		y += dy; x += dx;
		//CV_Assert((y >= 0 && y < rows) && (x >= 0 && x < cols));

		if (dy == -1)
		{
			linkPtr -= cols; mapPtr -= cols;
		}
		else
		{
			if (dy) //dy==1
			{
				linkPtr += cols; mapPtr += cols;
			}
		}
	}

	RemoveEdge(edge);

	return;
}

void bodyFilter::clearEdgeMap(EDGEMAP& edgeMap)
{
	EDGE* edges = edgeMap.edge;

	ushort edgeId;
	uchar child;
	int dy, dx;
	int y, x;
	uchar* linkPtr;
	ushort* mapPtr;

	int rows = edgeMap.link.rows;
	int cols = edgeMap.link.cols;

	int num = 0;
	int first = 0;
	int last = edgeMap.edgeNum - 1;
	while (last >= first)
	{
		EDGE& edge0 = edges[first];
		if (IsEdgeNotRemoved(edge0))
		{
			first++; num++;
			continue;
		}

		//find an edge
		for (; last > first; last--)
		{
			EDGE& edge1 = edges[last];
			if (IsEdgeNotRemoved(edge1))
				break;
		}

		if (last <= first)
			break;
		EDGE& edge1 = edges[last];
		last--;

		//move
		edgeId = first + 1;

		y = edge1.y0; x = edge1.x0;
		linkPtr = edgeMap.link.ptr<uchar>(y); mapPtr = edgeMap.map.ptr<ushort>(y);
		while (1)
		{
			mapPtr[x] = edgeId;

			child = Child(linkPtr[x]);
			if (!child)
				break;
			getChildDYX(child, dy, dx);
			y += dy; x += dx;
			//CV_Assert((y >= 0 && y < rows) && (x >= 0 && x < cols));

			if (dy == -1)
			{
				linkPtr -= cols; mapPtr -= cols;
			}
			else
			{
				if (dy) //dy==1
				{
					linkPtr += cols; mapPtr += cols;
				}
			}
		}

		edge0 = edge1;
		first++; num++;
	}

	edgeMap.edgeNum = num;
	return;
}

void bodyFilter::getConnectDyDx(Mat& link, EDGE& edge, int* dy, int* dx, int len)
{
	int y = edge.y1; int x = edge.x1;
	uchar* linkPtr = link.ptr<uchar>(y);

	int num = 0;

	//// the first point ////
	int dy1, dx1;
	getParentDYX(linkPtr[x], dy1, dx1);
	num++;

	y += dy1; x += dx1;
	if (dy1 == -1)
		linkPtr -= link.cols;
	else if (dy1) //dy1==1
		linkPtr += link.cols;

	dy1 = -dy1; dx1 = -dx1;

	//// the second point ////
	int dy0, dx0;
	uchar parent = Parent(linkPtr[x]);
	if (parent)
	{
		getParentDYX(parent, dy0, dx0);
		num++;

		dy0 = -dy0; dx0 = -dx0;
	}

	//// generate pattern ////
	if (num == 1)
	{
		//CV_Assert(dy1 == 1 || dx1 == 1);
		int* yPtr = dy; int* xPtr = dx;
		for (int i = 0; i < len; i++)
		{
			*(yPtr++) = dy1; *(xPtr++) = dx1;
		}

		return;
	}

	//num == 2
	//CV_Assert(dy0 == 1 && dy1 == 1 || dx0 == 1 && dx1 == 1);
	if (dy0 == 1 && dy1 == 1)
	{
		if (dx0 && dx0 == -dx1)
		{
			dx0 = 0; dx1 = 0;
		}
	}

	if (dx0 == 1 && dx1 == 1)
	{
		if (dy0 && dy0 == -dy1)
		{
			dy0 = 0; dy1 = 0;
		}
	}

	int* yPtr = dy; int* xPtr = dx;
	for (int i = 0; i < len / 2; i++)
	{
		*(yPtr++) = dy0; *(xPtr++) = dx0;
		*(yPtr++) = dy1; *(xPtr++) = dx1;
	}

	if (len % 2)
	{
		*(yPtr++) = dy0; *(xPtr++) = dx0;
	}

	return;
}

void bodyFilter::connectVEdgeMap(EDGEMAP& edgeMap, int len)
{
	EDGE* edges = edgeMap.edge;	ushort& edgesNum = edgeMap.edgeNum;
	Mat& link = edgeMap.link; Mat& map = edgeMap.map;

	if (len > MAX_CONNECT_LEN)
		len = MAX_CONNECT_LEN;

	int bridgeDy[MAX_CONNECT_LEN + 1], bridgeDx[MAX_CONNECT_LEN + 1];

	int len0;
	int y, x;
	ushort *mapPtr;
	ushort edgeId0, edgeId1;

	int rows = link.rows;
	int cols = link.cols;

	for (edgeId0 = 1; edgeId0 <= edgesNum; edgeId0++)
	{
		EDGE& edge0 = edges[edgeId0 - 1];

		if (edge0.y1 == edge0.y0) //has edge1 been removed?
			continue;

		getConnectDyDx(link, edge0, bridgeDy, bridgeDx, len + 1);

		//check
		y = edge0.y1; x = edge0.x1;
		mapPtr = map.ptr<ushort>(y);
		edgeId1 = 0;
		for (len0 = 0; len0 <= len; len0++)
		{
			//CV_Assert(bridgeDy[len0] == 1);
			y++;
			if (y >= rows)
				break;
			mapPtr += cols;

			if (mapPtr[x])
			{
				EDGE& edge = edges[mapPtr[x] - 1];
				if (edge.y0 == y && edge.x0 == x)
				{
					edgeId1 = mapPtr[x];
					bridgeDx[len0] = 0;
					break;
				}
			}

			if (x > 0)
			{
				int xx = x - 1;
				if (mapPtr[xx])
				{
					EDGE& edge = edges[mapPtr[xx] - 1];
					if (edge.y0 == y && edge.x0 == xx)
					{
						edgeId1 = mapPtr[xx];
						bridgeDx[len0] = -1;
						break;
					}
				}
			}

			if (x < (cols - 1))
			{
				int xx = x + 1;
				if (mapPtr[xx])
				{
					EDGE& edge = edges[mapPtr[xx] - 1];
					if (edge.y0 == y && edge.x0 == xx)
					{
						edgeId1 = mapPtr[xx];
						bridgeDx[len0] = 1;
						break;
					}
				}
			}

			x += bridgeDx[len0];
			if (x < 0 || x >= cols)
				break;
			if (mapPtr[x])
				break;
		}

		if (!edgeId1)
			continue;
		//CV_Assert(edgesNum > 1);

		//merge edge0 and edge1
		mergeEdges(edgeMap, edgeId0, edgeId1, bridgeDy, bridgeDx, len0 + 1);
	}

	clearEdgeMap(edgeMap);
	return;
}

void bodyFilter::connectHEdgeMap(EDGEMAP& edgeMap, int len)
{
	EDGE* edges = edgeMap.edge;	ushort& edgesNum = edgeMap.edgeNum;
	Mat& link = edgeMap.link; Mat& map = edgeMap.map;

	if (len > MAX_CONNECT_LEN)
		len = MAX_CONNECT_LEN;

	int bridgeDy[MAX_CONNECT_LEN + 1], bridgeDx[MAX_CONNECT_LEN + 1];

	int len0;
	int y, x;
	ushort *mapPtr, *mapPtr0, *mapPtr1;
	ushort edgeId0, edgeId1;

	int rows = link.rows;
	int cols = link.cols;

	for (edgeId0 = 1; edgeId0 <= edgesNum; edgeId0++)
	{
		EDGE& edge0 = edges[edgeId0 - 1];
		if (edge0.x1 == edge0.x0) //has edge1 been removed?
			continue;

		getConnectDyDx(link, edge0, bridgeDy, bridgeDx, len + 1);

		//check
		y = edge0.y1; x = edge0.x1;
		mapPtr = map.ptr<ushort>(y); mapPtr0 = mapPtr - cols; mapPtr1 = mapPtr + cols;
		edgeId1 = 0;
		for (len0 = 0; len0 <= len; len0++)
		{
			//CV_Assert(bridgeDx[len0] == 1);
			x++;
			if (x >= cols)
				break;

			if (mapPtr[x])
			{
				EDGE& edge = edges[mapPtr[x] - 1];
				if (edge.y0 == y && edge.x0 == x)
				{
					edgeId1 = mapPtr[x];
					bridgeDy[len0] = 0;
					break;
				}
			}

			if (y > 0)
			{
				if (mapPtr0[x])
				{
					EDGE& edge = edges[mapPtr0[x] - 1];
					if (edge.y0 == (y - 1) && edge.x0 == x)
					{
						edgeId1 = mapPtr0[x];
						bridgeDy[len0] = -1;
						break;
					}
				}
			}

			if (y < (rows - 1))
			{
				if (mapPtr1[x])
				{
					EDGE& edge = edges[mapPtr1[x] - 1];
					if (edge.y0 == (y + 1) && edge.x0 == x)
					{
						edgeId1 = mapPtr1[x];
						bridgeDy[len0] = 1;
						break;
					}
				}
			}

			int dy = bridgeDy[len0];
			y += dy;
			if (y < 0 || y >= rows)
				break;

			if (dy == -1)
			{
				mapPtr1 = mapPtr; mapPtr = mapPtr0; mapPtr0 -= cols;
			}
			else
			{
				if (dy) //dy==1
				{
					mapPtr0 = mapPtr; mapPtr = mapPtr1; mapPtr1 += cols;
				}
			}

			if (mapPtr[x])
				break;
		}

		if (!edgeId1)
			continue;
		//CV_Assert(edgesNum > 1);

		//merge edge0 and edge1
		mergeEdges(edgeMap, edgeId0, edgeId1, bridgeDy, bridgeDx, len0 + 1);
	}

	clearEdgeMap(edgeMap);
	return;
}

void bodyFilter::removeRedundantEdge(EDGEMAP& edgeMap)
{
	EDGE* edges = edgeMap.edge;	ushort& edgesNum = edgeMap.edgeNum;
	Mat& link = edgeMap.link; Mat& map = edgeMap.map;

	uchar child;
	int dy, dx;
	uchar *linkPtr, *linkPtr0, *linkPtr1;
	ushort *mapPtr, *mapPtr0, *mapPtr1;
	int y, x;
	ushort edgeId, edgeId0;

	int rows = link.rows;
	int cols = link.cols;

	for (edgeId0 = 1; edgeId0 <= edgesNum; edgeId0++)
	{
		EDGE& edge0 = edges[edgeId0 - 1];
		if (IsEdgeRemoved(edge0))
			continue;

		//check redundant edge
		y = edge0.y0; x = edge0.x0;
		mapPtr = map.ptr<ushort>(y); linkPtr = link.ptr<uchar>(y);
		mapPtr0 = mapPtr - cols; linkPtr0 = linkPtr - cols;
		mapPtr1 = mapPtr + cols; linkPtr1 = linkPtr + cols;
		while (1)
		{
			//left
			edgeId = 0;
			if (x > 0 && mapPtr[x - 1])
			{
				ushort id = mapPtr[x - 1];
				if (id != edgeId0)
				{
					if (IsEdgeNotRemoved(edges[id - 1]))
						edgeId = id;
				}
			}
			//right
			if (!edgeId && x < (cols - 1) && mapPtr[x + 1])
			{
				ushort id = mapPtr[x + 1];
				if (id != edgeId0)
				{
					if (IsEdgeNotRemoved(edges[id - 1]))
						edgeId = id;
				}
			}
			//up
			if (!edgeId && y > 0 && mapPtr0[x])
			{
				ushort id = mapPtr0[x];
				if (id != edgeId0)
				{
					if (IsEdgeNotRemoved(edges[id - 1]))
						edgeId = id;
				}
			}
			//down
			if (!edgeId && y < (rows - 1) && mapPtr1[x])
			{
				ushort id = mapPtr1[x];
				if (id != edgeId0)
				{
					if (IsEdgeNotRemoved(edges[id - 1]))
						edgeId = id;
				}
			}

			if (edgeId == 0)
				break;

			//move
			child = Child(linkPtr[x]);
			if (!child)
				break;
			getChildDYX(child, dy, dx);
			y += dy; x += dx;
			//CV_Assert((y >= 0 && y < rows) && (x >= 0 && x < cols));

			if (dy == -1)
			{
				linkPtr1 = linkPtr; linkPtr = linkPtr0; linkPtr0 -= cols;
				mapPtr1 = mapPtr;  mapPtr = mapPtr0;  mapPtr0 -= cols;
			}
			else
			{
				if (dy) //dy==1
				{
					linkPtr0 = linkPtr; linkPtr = linkPtr1; linkPtr1 += cols;
					mapPtr0 = mapPtr;  mapPtr = mapPtr1;   mapPtr1 += cols;
				}
			}
		}

		if (edgeId)
			clearEdge(edgeMap, edgeId0);//clear edge0
	}

	clearEdgeMap(edgeMap);
	return;
}
