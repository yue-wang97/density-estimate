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
///////////////////////////////////HOG+SVMʶ��ʽ1///////////////////////////////////////////////////

void HOG_SVM1()

{

	////////////////////////////////����ѵ������ͼƬ·�������///////////////////////////////////////////////////

	//ͼ��·�������

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

				//��ȡ�������

				imageClass.push_back(atoi(buffer.c_str()));

			}

			else

			{

				//��ȡͼ��·��

				imagePath.push_back(buffer);

			}

		}

	}

	trainingData.close();





	////////////////////////////////��ȡ������HOG����///////////////////////////////////////////////////

	//����������������

	int numberOfSample = numberOfLine / 2;

	Mat featureVectorOfSample(numberOfSample, 3780, CV_32FC1);//������ÿ��Ϊһ������



															  //���������

	Mat classOfSample(numberOfSample, 1, CV_32SC1);



	//��ʼ����ѵ��������HOG����

	for (vector<string>::size_type i = 0; i <= imagePath.size() - 1; ++i)

	{

		//����ͼƬ

		Mat src = imread(imagePath[i], -1);

		if (src.empty())

		{

			cout << "can not load the image:" << imagePath[i] << endl;

			continue;

		}

		cout << "processing" << imagePath[i] << endl;



		//����

		Mat trainImage;

		resize(src, trainImage, Size(64, 128));



		//��ȡHOG����

		HOGDescriptor hog(Size(64, 128), Size(16, 16), Size(8, 8), Size(8, 8), 9);

		vector<float> descriptors;

		hog.compute(trainImage, descriptors);//����������ü�ⴰ�ڲ��������ͼƬ��С����64��128����������winStride

		cout << "HOG dimensions:" << descriptors.size() << endl;



		//������������������

		for (vector<float>::size_type j = 0; j <= descriptors.size() - 1; ++j)

		{

			featureVectorOfSample.at<float>(i, j) = descriptors[j];

		}



		//�������������

		//!!ע���������һ��Ҫ��int ���͵�

		classOfSample.at<int>(i, 0) = imageClass[i];

	}





	///////////////////////////////////ʹ��SVM������ѵ��///////////////////////////////////////////////////    

	//���ò���

	//�ο�3.0��Demo

	Ptr<SVM> svm = SVM::create();

	svm->setKernel(SVM::RBF);

	svm->setType(SVM::C_SVC);

	svm->setC(10);

	svm->setCoef0(1.0);

	svm->setP(1.0);

	svm->setNu(0.5);

	svm->setTermCriteria(TermCriteria(CV_TERMCRIT_EPS, 1000, FLT_EPSILON));



	//ʹ��SVMѧϰ         

	svm->train(featureVectorOfSample, ROW_SAMPLE, classOfSample);



	//���������

	svm->save("Classifier.xml");





	///////////////////////////////////ʹ��ѵ���õķ���������ʶ��///////////////////////////////////////////////////

	vector<string> testImagePath;

	ifstream testData(string(FILEPATH) + "TestData.txt", ios::out);

	while (!testData.eof())

	{

		getline(testData, buffer);

		//��ȡ

		if (!buffer.empty())

			testImagePath.push_back(buffer);



	}

	testData.close();



	ofstream fileOfPredictResult(string(FILEPATH) + "PredictResult.txt"); //���ʶ��Ľ��

	for (vector<string>::size_type i = 0; i <= testImagePath.size() - 1; ++i)

	{

		//��ȡ����ͼƬ

		Mat src = imread(testImagePath[i], -1);

		if (src.empty())

		{

			cout << "Can not load the image:" << testImagePath[i] << endl;

			continue;

		}



		//����

		Mat testImage;

		resize(src, testImage, Size(64, 64));



		//����ͼƬ��ȡHOG����

		HOGDescriptor hog(cvSize(64, 64), cvSize(16, 16), cvSize(8, 8), cvSize(8, 8), 9);

		vector<float> descriptors;

		hog.compute(testImage, descriptors);

		cout << "HOG dimensions:" << descriptors.size() << endl;



		Mat featureVectorOfTestImage(1, descriptors.size(), CV_32FC1);

		for (int j = 0; j <= descriptors.size() - 1; ++j)

		{

			featureVectorOfTestImage.at<float>(0, j) = descriptors[j];

		}



		//�Բ���ͼƬ���з��ಢд���ļ�

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

	//HOG+SVMʶ��ʽ1��ֱ��������

	HOG_SVM1();



	//HOG+SVMʶ��ʽ2�����ͼƬ�еĴ���Ŀ��ľ���

	//HOG_SVM2();

