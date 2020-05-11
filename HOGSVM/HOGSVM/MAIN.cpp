#include "opencv2/opencv.hpp"
#include "opencv2/ml.hpp"
#include "opencv2/imgproc.hpp"

#include <stdio.h>
#include <string.h>
#include <cctype>
#include<iostream>
#include <fstream>

using namespace cv;
using namespace cv::ml;
using namespace std;

#define INPUT_VIDEO_1  "D:\\study\\myproject\\graduate\\myvideo\\carFew.mp4"
#define INPUT_VIDEO_2  "D:\\study\\myproject\\graduate\\myvideo\\carMany.mp4"
#define INPUT_VIDEO_3  "D:\\study\\myproject\\graduate\\myvideo\\crowd.mp4"
#define INPUT_VIDEO_4  "D:\\study\\myproject\\graduate\\myvideo\\hengCar.mp4"
#define INPUT_VIDEO_5  "D:\\study\\myproject\\graduate\\myvideo\\people.mp4"

#define ALARM_THRESHOLD			0.1 //实验表明：取值应当在0.1～0.20之间

///////////////////////////////////HOG+SVM识别方式2///////////////////////////////////////////////////	
// 使用训练好的分类器识别
void Detect()
{
	Mat img;
	VideoCapture capture(INPUT_VIDEO_5);
	if (!capture.isOpened()) {
		cout << "fail to open!" << endl;
	}



	//加载训练好的判别函数的参数(注意，与svm->save保存的分类器不同)
	vector<float> detector;


	//设置HOG
	HOGDescriptor hog;
	hog.setSVMDetector(HOGDescriptor::getDefaultPeopleDetector());//可以直接使用05 CVPR已训练好的分类器,这样就不用Train()这个步骤了
	namedWindow("people detector", 1);

	// 检测图片
	////（2）输入一组视频图像（假设视频图像个数是20）////
	size_t i, j;
	for (int i = 0; i < 20; i++) //???????????
	{
		capture >> img;


		fflush(stdout);
		vector<Rect> found, found_filtered;
		double t = (double)getTickCount();
		// run the detector with default parameters. to get a higher hit-rate
		// (and more false alarms, respectively), decrease the hitThreshold and
		// groupThreshold (set groupThreshold to 0 to turn off the grouping completely).
		//多尺度检测
		hog.detectMultiScale(img, found, 0, Size(8, 8), Size(32, 32), 1.05, 2);
		t = (double)getTickCount() - t;
		printf("detection time = %gms\n", t*1000. / cv::getTickFrequency());

		//去掉空间中具有内外包含关系的区域，保留大的
		for (i = 0; i < found.size(); i++)
		{
			Rect r = found[i];
			for (j = 0; j < found.size(); j++)
				if (j != i && (r & found[j]) == r)
					break;
			if (j == found.size())
				found_filtered.push_back(r);
		}

		// 适当缩小矩形
		for (i = 0; i < found_filtered.size(); i++)
		{
			Rect r = found_filtered[i];
			// the HOG detector returns slightly larger rectangles than the real objects.
			// so we slightly shrink the rectangles to get a nicer output.
			r.x += cvRound(r.width*0.1);
			r.width = cvRound(r.width*0.8);
			r.y += cvRound(r.height*0.07);
			r.height = cvRound(r.height*0.8);
			rectangle(img, r.tl(), r.br(), cv::Scalar(0, 255, 0), 3);
		}

		imshow("people detector", img);
		int c = waitKey(0) & 255;

	}

	return;
}

void HOG_SVM2()
{
	//如果使用05 CVPR提供的默认分类器，则不需要Train(),直接使用Detect检测图片
	//Train();
	Detect();
}

#define PATH "D:/Pedestrians64x128/"

int main()
{
	//HOG+SVM识别方式1：直接输出类别
	//HOG_SVM1();

	//HOG+SVM识别方式2：在图片中标识出存在目标的矩形
	HOG_SVM2();

}