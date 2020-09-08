#include <iostream>
#include <opencv2/opencv.hpp>

int main()
{
	cv::Mat left = cv::imread("D:/workspace/LF/LFCameraCalib/out/build/x64-Release/LFRendering/10_04.bmp"), 
		right = cv::imread("D:/workspace/LF/LFCameraCalib/out/build/x64-Release/LFRendering/10_15.bmp");
	
	cv::blur(left, left, cv::Size(5, 5));
	cv::medianBlur(left, left, 5);
	cv::blur(right, right, cv::Size(5, 5));
	cv::medianBlur(right, right, 5);
	std::vector<cv::Mat> imgs;
	cv::Mat img;
	for (int i = 0; i < 20; ++i)
	{
		char name[128]{ 0 };
		sprintf(name, "D:/workspace/LF/LFCameraCalib/out/build/x64-Release/LFRendering/10_%02d.bmp", i);
		img = cv::imread(name);
		imgs.push_back(img);
	}

	cv::Mat line;

	int row, col, pixelRow = 300;

	while (true)
	{
		std::cout << "Line: " << pixelRow << "\n";
		line = cv::Mat();
		
		for (auto& i : imgs)
			line.push_back(i.row(pixelRow));
		cv::GaussianBlur(line, line, cv::Size(5, 5), 0.7);
		cv::cvtColor(line, line, cv::COLOR_BGR2GRAY);
		cv::imshow("Line", line);

		imgs[10].copyTo(img);
		img.row(pixelRow) = cv::Scalar(0, 0, 255);
		cv::imshow("IMG", img);

		int key = cv::waitKeyEx();
		if (key == 27)
			break;
		else if (key == 0x00250000)
		{
			pixelRow = std::max(0, pixelRow - 1);
		}
		else if (key == 0x00270000)
		{
			pixelRow = std::min(imgs[0].rows - 1, pixelRow + 1);
		}
		else if (key == 's')
		{
			cv::imwrite("View.bmp", img);
			cv::imwrite("Line.bmp", line);
		}
	}

	std::cout << left.size() << "\n" << right.size() << "\n";
	cv::Ptr<cv::StereoMatcher> sm = cv::StereoSGBM::create(0, 50, 15);
	cv::Mat disparity;
	sm->compute(left, right, disparity);
	cv::normalize(disparity, disparity, 0, 255, cv::NORM_MINMAX);
	disparity.convertTo(disparity, CV_8U);
	cv::imshow("left", left);
	cv::imshow("right", right);
	cv::imshow("Disparity", disparity);
	cv::waitKey();
}