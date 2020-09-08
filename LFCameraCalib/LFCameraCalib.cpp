#include <afxwin.h>
#include <opencv2/opencv.hpp>
#include <algorithm>
#include <iostream>

#include "calib_grid.hpp"
#include "MVCAPI.h"

using namespace wd;

void onChange(int pos, void* userdata)
{
	*(bool*)(userdata) = true;
}


int main(int argc, char** argv)
{
	//if (argc != 2)
	//{
	//	std::cout << "Usage: DetectGrid <full image path>\n";
	//	return 1;
	//}
	bool refresh = true;


	cv::Mat src_img_bgr;
	cv::Mat src_img;
	cv::Mat gridMat;

	cv::Mat threshMat;
	cv::Mat imgShow;
	int medianBlurSize = 7, blockSize = 31, C = 1, openOpBlockSize = 13;
	int left(60), right(60), top(60), bottom(90);

	const cv::Scalar sc[3]{ cv::Scalar(255, 0, 0), cv::Scalar(0, 255, 0), cv::Scalar(0, 0, 255) };


	cv::namedWindow("Parameter");
	cv::createTrackbar("medianBlurSize", "Parameter", &medianBlurSize, 60, onChange, &refresh);
	cv::createTrackbar("blockSize", "Parameter", &blockSize, 60, onChange, &refresh);
	cv::createTrackbar("C", "Parameter", &C, 60, onChange, &refresh);
	cv::createTrackbar("enrodBlockSize", "Parameter", &openOpBlockSize, 60, onChange, &refresh);

	cv::createTrackbar("left", "Parameter", &left, 100);
	cv::createTrackbar("right", "Parameter", &right, 100);
	cv::createTrackbar("top", "Parameter", &top, 100);
	cv::createTrackbar("bottom", "Parameter", &bottom, 100);

	static std::vector<std::vector<cv::Point2d>> grid;

	char key = 0, mode = 'e';

	int devNo(0);
	int devQty = MVC_GetDeviceNumber();
	std::cout << devQty << "\n";
	MVCGE_DEVLISTEX DevInfo;
	for (int i = 0; i < devQty; ++i)
	{
		MVC_GetDeviceInfoEx(i, &DevInfo);
		std::cout << std::setw(20) << std::left << DevInfo.AdapterIP << std::setw(20) << std::right << DevInfo.DevMAC << "\n";
	}
	std::cout << "Open device: " << MVC_OpenDevice(devNo) << "\n";
	MVC_AutoWhiteBalance(0);

	double gain = MVC_GetParameter(0, MVCADJ_DACGAIN), exposure = MVC_GetParameter(0, MVCADJ_INTTIME);
	// MVC_SetNetPacketSize(devNo, 1440);

	MVC_EnableCapture(devNo);

	cv::Mat frameMat(3288, 4384, CV_8UC1), frameMatBGR(3288, 4384, CV_8UC3);
	MVCFRAMEINFO Frame;

	cv::Mat GridMatrix;
	int GridRows, GridCols;

	while (true)
	{
		{
			// std::cout << "Grab image\n";
			std::cout << "Get raw data: " << MVC_GetRawData(devNo, &Frame) << "\n";
			std::cout << std::setw(10) << std::left << Frame.FRAMEID << std::setw(4) << ": [" << Frame.Width << ", " << Frame.Height << "] " << Frame.lBufSize << "\n";
			std::cout << "Pixel type: " << Frame.PixelID << "\n";
			// frameMat.data = Frame.lBufPtr;
			memcpy(frameMat.data, Frame.lBufPtr, Frame.lBufSize);
			cv::cvtColor(frameMat, frameMatBGR, cv::COLOR_BayerGB2BGR);
			cv::cvtColor(frameMatBGR, src_img, cv::COLOR_BGR2GRAY);
			cv::pyrDown(frameMatBGR, gridMat);
			// cv::pyrDown(gridMat, gridMat);
			cv::imshow("src_img", gridMat);
			// cv::waitKey();
		}
		if (refresh)
		{
			std::cout << "Render one frame\n";
			refresh = false;
			cv::medianBlur(src_img, threshMat, medianBlurSize * 2 + 1);
			cv::adaptiveThreshold(threshMat, threshMat, 255, cv::ADAPTIVE_THRESH_GAUSSIAN_C, cv::THRESH_BINARY, blockSize * 2 + 3, static_cast<double>(C) / 100.0);
			cv::morphologyEx(threshMat, threshMat, cv::MORPH_OPEN, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(openOpBlockSize * 2 + 1, openOpBlockSize * 2 + 1)));

			cv::pyrDown(threshMat, imgShow);
			cv::imshow("proc", imgShow);
		}
		key = cv::waitKey(1);
		if (key == 27)
			break;
		else if (key == 'p')
		{
			grid.clear();

			std::vector<cv::Point2d> centers;
			detectCenters(threshMat, centers);
			//cv::pyrDown(frameMatBGR, gridMat);
			centers = cropBorder(centers, src_img.cols, src_img.rows, left, right, top, bottom);
			std::cout << centers.size() << "\n";

			grid = detectGrid(centers);

			for (int i = 0; i < grid.size(); ++i)
			{
				cv::rectangle(gridMat, cv::Rect(grid[i][0].x / 2 - 5, grid[i][0].y / 2 - 5, 10, 10), sc[i % 3], 2);
				for (int j = 1; j < grid[i].size(); ++j)
				{
					cv::line(gridMat, grid[i][j] / 2.0, grid[i][j - 1LL] / 2.0, sc[i % 3]);
					cv::circle(gridMat, grid[i][j] / 2.0, 5, sc[i % 3], 2);
				}
			}

			cv::imshow("Grid", gridMat);
		}
		else if (key == 'o')
		{
			if (grid.empty() || gridMat.empty())
				continue;
			std::vector<cv::Point2f> points;
			for (auto& i : grid)
				for (auto& j : i)
					points.push_back(cv::Point2f(j.x, j.y));
			cv::Mat x, y;
			solveGrid(points, grid[0].size(), grid.size(), x, y);
			std::cout << x << "\n" << y << "\n";
			GridMatrix = (cv::Mat_<double>(2, 3) << x.at<double>(0), x.at<double>(1), x.at<double>(2), y.at<double>(0), y.at<double>(1), y.at<double>(2));
			GridRows = grid.size();
			GridCols = grid[0].size();
			for (size_t i = 0; i < grid.size(); ++i)
			{
				for (size_t j = 0; j < grid[0].size(); ++j)
				{
					cv::Mat v = (cv::Mat_<double>(1, 3) << 1, j, i);
					cv::Point p;
					p.x = cv::Mat(v * x).at<double>(0);
					p.y = cv::Mat(v * y).at<double>(0);
					cv::circle(gridMat, p / 2, 5, sc[(i + 1) % 3], 5);
				}
			}

			cv::imshow("Grid", gridMat);
		}
		else if (key == 's')
		{
			cv::FileStorage calib_config(std::string("D:/workspace/LFCameraCalib/camera_calibration.yaml"), cv::FileStorage::WRITE);
			calib_config << "GridRows " << GridRows;
			calib_config << "GridCols " << GridCols;
			calib_config << "GridMatrix " << GridMatrix;
			calib_config.release();
		}
		else if (key == '0')
			refresh = true;
		else if (key == 'e' || key == 'g')
		{
			mode = key;
		}
		else if (key == ',')
		{
			if (mode == 'e')
			{
				exposure = std::max(0.0, exposure - 100);
				MVC_SetParameter(0, MVCADJ_INTTIME, exposure);
				std::cout << "Exposure: " << exposure << "\n";
			}
			else if (mode == 'g')
			{
				gain = std::max(0.0, gain - 5);
				MVC_SetParameter(0, MVCADJ_DACGAIN, gain);
				std::cout << "Gain: " << gain << "\n";
			}
		}
		else if (key == '.')
		{
			if (mode == 'e')
			{
				exposure = std::min(4000., exposure + 100);
				MVC_SetParameter(0, MVCADJ_INTTIME, exposure);
				std::cout << "Exposure: " << exposure << "\n";
			}
			else if (mode == 'g')
			{
				gain = std::min(120.0, gain + 5);
				MVC_SetParameter(0, MVCADJ_DACGAIN, gain);
				std::cout << "Gain: " << gain << "\n";
			}
		}
		else if (key == 'a')
		{
			std::cout << "Auto white balance " << MVC_AutoWhiteBalance(static_cast<DWORD>(0)) << "\n";
		}
	}
	MVC_DisableCapture(devNo);
	MVC_CloseDevice(devNo);
	return 0;
}