// readImg.cpp : �������̨Ӧ�ó������ڵ㡣
//

#include "stdafx.h"
#include <opencv2\opencv.hpp>
#include "opencv2/imgproc.hpp"
#include <iostream>
#include <string>

#include "PedestrianDetection.h"

using namespace std;
using namespace cv;

#define INPUT_VIDEO_1  "D:\\study\\myproject\\graduate\\myvideo\\carFew.mp4"
#define INPUT_VIDEO_2  "D:\\study\\myproject\\graduate\\myvideo\\fuzimiao.mp4"
#define INPUT_VIDEO_3  "D:\\study\\myproject\\graduate\\myvideo\\crowd.mp4"
#define INPUT_VIDEO_4  "D:\\study\\myproject\\graduate\\myvideo\\hengCar.mp4"
#define INPUT_VIDEO_5  "D:\\study\\myproject\\graduate\\myvideo\\people.mp4"
#define INPUT_VIDEO_6  "C:\\Users\\hxp\\Desktop\\Programming\\Nanjing\\data\\913-37_l.mov"
#define INPUT_VIDEO_7  "C:\\Users\\hxp\\Desktop\\Programming\\Nanjing\\data\\CUHKSquare.mpg"
#define INPUT_VIDEO_8  "C:\\Users\\hxp\\Desktop\\Programming\\Nanjing\\data\\grandcentral.avi"
#define INPUT_VIDEO_9  "C:\\Users\\hxp\\Desktop\\Programming\\Nanjing\\data\\88bOOOPIC37.mp4"
#define INPUT_VIDEO_10  "C:\\Users\\hxp\\Desktop\\Programming\\Nanjing\\data\\20160706082424544848517_1_06400360.mp4"

#define INPUT_VIDEO_30  "C:\\Users\\hxp\\Desktop\\Programming\\Nanjing\\data\\nanjing-1.mp4"
#define INPUT_VIDEO_31  "C:\\Users\\hxp\\Desktop\\Programming\\Nanjing\\data\\nanjing-2.mp4"
#define INPUT_VIDEO_32  "C:\\Users\\hxp\\Desktop\\Programming\\Nanjing\\data\\nanjing-record1.mp4"
#define INPUT_VIDEO_33  "C:\\Users\\hxp\\Desktop\\Programming\\Nanjing\\data\\nanjing-record2.mp4"


#define ALARM_THRESHOLD			0.1 //ʵ�������ȡֵӦ����0.1��0.20֮��

int main()
{
	VideoCapture capture(INPUT_VIDEO_5);
	if (!capture.isOpened()) {
		cout << "fail to open!" << endl;
	}

	///////////////////////////////////////////////////////////
	PedestrianDetection PedestrianDetection;

	////��1����������////
	PedestrianParameters parameters;
	PedestrianDetection.getParameters(parameters);
	//�޸Ĳ���......
	PedestrianDetection.setParameters(parameters);

	////��2������һ����Ƶͼ�񣨼�����Ƶͼ�������20��////
	Mat frame;
	//for(int i = 0; i < 20; i++)
	for (int i = 0; i < 26; i++) //???????????
	{
		capture >> frame;

		PedestrianDetection.inputFrame(frame);


		////��3����ȡ���////
		int pedestrianNum, pedestrianArea;
		Mat pedestrianAreaMask;
		if (PedestrianDetection.getResult(pedestrianNum, pedestrianArea, pedestrianAreaMask))
		{
			double pedestrianAreaR = double(pedestrianArea) / double(frame.rows * frame.cols);
			//if (pedestrianAreaR > ALARM_THRESHOLD) //������ռ���������10%��20%ʱ����
			//	sendAlarm();

			//��ʾ�ʹ�ӡ����������ã�
			PedestrianDetection.showMask("PedestrianArea", frame, pedestrianAreaMask, false);
			printf("areaSize = %d (%f), body=%d\n", pedestrianArea, double(pedestrianArea) / double(frame.rows*frame.cols), pedestrianNum);
			waitKey(1);
		}
		waitKey(1);
	}
	return 0;

}
