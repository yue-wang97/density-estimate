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

#define ALARM_THRESHOLD			0.1 //ʵ�������ȡֵӦ����0.1��0.20֮��

///////////////////////////////////HOG+SVMʶ��ʽ2///////////////////////////////////////////////////	
// ʹ��ѵ���õķ�����ʶ��
void Detect()
{
	Mat img;
	VideoCapture capture(INPUT_VIDEO_5);
	if (!capture.isOpened()) {
		cout << "fail to open!" << endl;
	}



	//����ѵ���õ��б����Ĳ���(ע�⣬��svm->save����ķ�������ͬ)
	vector<float> detector;


	//����HOG
	HOGDescriptor hog;
	hog.setSVMDetector(HOGDescriptor::getDefaultPeopleDetector());//����ֱ��ʹ��05 CVPR��ѵ���õķ�����,�����Ͳ���Train()���������
	namedWindow("people detector", 1);

	// ���ͼƬ
	////��2������һ����Ƶͼ�񣨼�����Ƶͼ�������20��////
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
		//��߶ȼ��
		hog.detectMultiScale(img, found, 0, Size(8, 8), Size(32, 32), 1.05, 2);
		t = (double)getTickCount() - t;
		printf("detection time = %gms\n", t*1000. / cv::getTickFrequency());

		//ȥ���ռ��о������������ϵ�����򣬱������
		for (i = 0; i < found.size(); i++)
		{
			Rect r = found[i];
			for (j = 0; j < found.size(); j++)
				if (j != i && (r & found[j]) == r)
					break;
			if (j == found.size())
				found_filtered.push_back(r);
		}

		// �ʵ���С����
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
	//���ʹ��05 CVPR�ṩ��Ĭ�Ϸ�����������ҪTrain(),ֱ��ʹ��Detect���ͼƬ
	//Train();
	Detect();
}

#define PATH "D:/Pedestrians64x128/"

int main()
{
	//HOG+SVMʶ��ʽ1��ֱ��������
	//HOG_SVM1();

	//HOG+SVMʶ��ʽ2����ͼƬ�б�ʶ������Ŀ��ľ���
	HOG_SVM2();

}