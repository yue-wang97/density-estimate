#include "opencv2/opencv.hpp"
#include "opencv2/ml.hpp"

#include <stdio.h>
#include <string.h>
#include <cctype>
#include<iostream>
#include <fstream>

using namespace cv;
using namespace cv::ml;
using namespace std;

#define FILEPATH  "D:/Pedestrians64x128/"
///////////////////////////////////HOG+SVM识别方式1///////////////////////////////////////////////////

void HOG_SVM1()

{

	////////////////////////////////读入训练样本图片路径和类别///////////////////////////////////////////////////

	//图像路径和类别

	vector<string> imagePath;

	vector<int> imageClass;

	int numberOfLine = 0;

	string buffer;

	ifstream trainingData(string(FILEPATH) + "TrainData.txt", ios::in);

	unsigned long n;



	while (!trainingData.eof())

	{

		getline(trainingData, buffer);

		if (!buffer.empty())

		{

			++numberOfLine;

			if (numberOfLine % 2 == 0)

			{

				//读取样本类别

				imageClass.push_back(atoi(buffer.c_str()));

			}

			else

			{

				//读取图像路径

				imagePath.push_back(buffer);

			}

		}

	}

	trainingData.close();





	////////////////////////////////获取样本的HOG特征///////////////////////////////////////////////////

	//样本特征向量矩阵

	int numberOfSample = numberOfLine / 2;

	Mat featureVectorOfSample(numberOfSample, 3780, CV_32FC1);//矩阵中每行为一个样本



															  //样本的类别

	Mat classOfSample(numberOfSample, 1, CV_32SC1);



	//开始计算训练样本的HOG特征

	for (vector<string>::size_type i = 0; i <= imagePath.size() - 1; ++i)

	{

		//读入图片

		Mat src = imread(imagePath[i], -1);

		if (src.empty())

		{

			cout << "can not load the image:" << imagePath[i] << endl;

			continue;

		}

		cout << "processing" << imagePath[i] << endl;



		//缩放

		Mat trainImage;

		resize(src, trainImage, Size(64, 128));



		//提取HOG特征

		HOGDescriptor hog(Size(64, 128), Size(16, 16), Size(8, 8), Size(8, 8), 9);

		vector<float> descriptors;

		hog.compute(trainImage, descriptors);//这里可以设置检测窗口步长，如果图片大小超过64×128，可以设置winStride

		cout << "HOG dimensions:" << descriptors.size() << endl;



		//保存特征向量矩阵中

		for (vector<float>::size_type j = 0; j <= descriptors.size() - 1; ++j)

		{

			featureVectorOfSample.at<float>(i, j) = descriptors[j];

		}



		//保存类别到类别矩阵

		//!!注意类别类型一定要是int 类型的

		classOfSample.at<int>(i, 0) = imageClass[i];

	}





	///////////////////////////////////使用SVM分类器训练///////////////////////////////////////////////////    

	//设置参数

	//参考3.0的Demo

	Ptr<SVM> svm = SVM::create();

	svm->setKernel(SVM::RBF);

	svm->setType(SVM::C_SVC);

	svm->setC(10);

	svm->setCoef0(1.0);

	svm->setP(1.0);

	svm->setNu(0.5);

	svm->setTermCriteria(TermCriteria(CV_TERMCRIT_EPS, 1000, FLT_EPSILON));



	//使用SVM学习         

	svm->train(featureVectorOfSample, ROW_SAMPLE, classOfSample);



	//保存分类器

	svm->save("Classifier.xml");





	///////////////////////////////////使用训练好的分类器进行识别///////////////////////////////////////////////////

	vector<string> testImagePath;

	ifstream testData(string(FILEPATH) + "TestData.txt", ios::out);

	while (!testData.eof())

	{

		getline(testData, buffer);

		//读取

		if (!buffer.empty())

			testImagePath.push_back(buffer);



	}

	testData.close();



	ofstream fileOfPredictResult(string(FILEPATH) + "PredictResult.txt"); //最后识别的结果

	for (vector<string>::size_type i = 0; i <= testImagePath.size() - 1; ++i)

	{

		//读取测试图片

		Mat src = imread(testImagePath[i], -1);

		if (src.empty())

		{

			cout << "Can not load the image:" << testImagePath[i] << endl;

			continue;

		}



		//缩放

		Mat testImage;

		resize(src, testImage, Size(64, 64));



		//测试图片提取HOG特征

		HOGDescriptor hog(cvSize(64, 64), cvSize(16, 16), cvSize(8, 8), cvSize(8, 8), 9);

		vector<float> descriptors;

		hog.compute(testImage, descriptors);

		cout << "HOG dimensions:" << descriptors.size() << endl;



		Mat featureVectorOfTestImage(1, descriptors.size(), CV_32FC1);

		for (int j = 0; j <= descriptors.size() - 1; ++j)

		{

			featureVectorOfTestImage.at<float>(0, j) = descriptors[j];

		}



		//对测试图片进行分类并写入文件

		int predictResult = svm->predict(featureVectorOfTestImage);

		char line[512];

		//printf("%s %d\r\n", testImagePath[i].c_str(), predictResult);

		std::sprintf(line, "%s %d\n", testImagePath[i].c_str(), predictResult);

		fileOfPredictResult << line;



	}

	fileOfPredictResult.close();

}



int main()

{

	//HOG+SVM识别方式1：直接输出类别

	HOG_SVM1();



	//HOG+SVM识别方式2：输出图片中的存在目标的矩形

	//HOG_SVM2();

