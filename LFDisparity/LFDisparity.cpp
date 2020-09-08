#include <afxwin.h>
#include <MVCAPI.h>

#include <opencv2/opencv.hpp>

void getSourceFrameCorner(const cv::Mat& GridMatrix, std::vector<cv::Point2f>& quads, int rows, int cols);

int main(int argc, char** argv)
{
	std::string calib_path = "D:/workspace/LF/LFCameraCalib/camera_calibration.yaml";

	cv::FileStorage calib_config(calib_path, cv::FileStorage::READ);
	cv::Mat GridMatrix;
	int GridRows(calib_config["GridRows"]), GridCols(calib_config["GridCols"]);
	calib_config["GridMatrix"] >> GridMatrix;

	cv::Mat src_img_bgr = cv::imread("D:/workspace/LF/LFCameraCalib/out/build/x64-Release/LFDisparity/1.bmp"), dst_img;

	std::vector<cv::Point2f> from, to;
	getSourceFrameCorner(GridMatrix, from, GridRows, GridCols);

	cv::Rect rect = cv::boundingRect(from);
	rect.width = 116 * GridCols;
	rect.height = 116 * GridRows;
	dst_img = cv::Mat::zeros(rect.size(), CV_8UC3);
	to.push_back(cv::Point2f(0, 0));
	to.push_back(cv::Point2f(0, rect.height - 1));
	to.push_back(cv::Point2f(rect.width - 1, rect.height - 1));
	to.push_back(cv::Point2f(rect.width - 1, 0));
	cv::Mat M = cv::getPerspectiveTransform(from, to);
	cv::warpPerspective(src_img_bgr, dst_img, M, dst_img.size());
	//cv::pyrDown(dst_img, dst_img);
	cv::imshow("Matrix", dst_img);

	std::cout << dst_img.size() << "\n";

	//for (auto& i : from)
	//{
	//	cv::circle(src_img_bgr, i, 10, cv::Scalar(0, 0, 255), 2);
	//}
	//cv::pyrDown(src_img_bgr, src_img_bgr);
	//cv::imshow("Points", src_img_bgr);
	cv::waitKey();

	int key = 0;
	int row(0), col(1), pixelRow(0);
	volatile bool isOk = true;
	rect.width = 116;
	rect.height = 116;
	cv::Mat line;
	static cv::Mat left, right, disparity;
	double minPixel, maxPixel;

	cv::Ptr<cv::StereoMatcher> sm = cv::StereoSGBM::create(0, 16, 3);

	cv::Mat epiDisparity = cv::Mat::zeros(dst_img.size(), CV_8U);
	while (isOk)
	{
		rect.x = (col - 1) * rect.width;
		rect.y = row * rect.height;
		left = dst_img(rect);
		//cv::cvtColor(dst_img(rect), left, cv::COLOR_BGR2GRAY);
		cv::medianBlur(left, left, 7);
		//cv::Canny(left, left, 30, 30);
		//cv::minMaxIdx(left, &minPixel, &maxPixel);
		//left = (left - minPixel) / ((maxPixel - minPixel) / 255.);
		cv::imshow("Left", left);
		rect.x = col * rect.width;
		rect.y = row * rect.height;
		right = dst_img(rect);
		//cv::cvtColor(dst_img(rect), right, cv::COLOR_BGR2GRAY);
		cv::medianBlur(right, right, 7);
		//cv::Canny(right, right, 30, 30);
		//right = (right - minPixel) / ((maxPixel - minPixel) / 255.);
		cv::imshow("Right", right);
		sm->compute(left, right, disparity);
		if (!disparity.empty())
		{
			//std::cout << disparity.channels() << "\t" << disparity.type() << "\n";
			//cv::minMaxIdx(disparity, &minPixel, &maxPixel);
			//std::cout << minPixel << "\t" << maxPixel << "\n"; 
			//disparity = (disparity - minPixel) / ((maxPixel - minPixel) / 255.);
			disparity = disparity / 16;
			disparity.convertTo(disparity, CV_8U);
			cv::equalizeHist(disparity, disparity);
			cv::imshow("Disparity", disparity);
		}
		line = dst_img.row(116 * row + pixelRow);
		line = line.reshape(3, GridCols);
		cv::pyrUp(line, line);
		cv::pyrUp(line, line);
		cv::imshow("EPI", line);



		key = cv::waitKeyEx();
		std::cout << rect << "\t" << pixelRow << "\n";
		printf("%08X\n", key);
		switch (key)
		{
		case 'w':
			row = std::max(0, row - 1);
			break;
		case 's':
			row = std::min(GridRows - 1, row + 1);
			break;
		case 'a':
			col = std::max(1, col - 1);
			break;
		case 'd':
			col = std::min(GridCols - 1, col + 1);
			break;
		case 27:
			isOk = false;
			break;
		case 0x00260000:
			pixelRow = std::max(0, pixelRow - 1);
			break;
		case 0x00280000:
			pixelRow = std::min(115, pixelRow + 1);
			break;
		default:
			break;
		}
	}

}

void getSourceFrameCorner(const cv::Mat& GridMatrix, std::vector<cv::Point2f>& quads, int rows, int cols)
{
	cv::Mat pMat;
	pMat = GridMatrix * (cv::Mat_<double>(3, 1) << 1, -0.5, -0.5);
	quads.push_back(cv::Point2f(pMat.at<double>(0), pMat.at<double>(1)));
	pMat = GridMatrix * (cv::Mat_<double>(3, 1) << 1, -0.5, rows - 0.5);
	quads.push_back(cv::Point2f(pMat.at<double>(0), pMat.at<double>(1)));
	pMat = GridMatrix * (cv::Mat_<double>(3, 1) << 1, cols - 0.5, rows - 0.5);
	quads.push_back(cv::Point2f(pMat.at<double>(0), pMat.at<double>(1)));
	pMat = GridMatrix * (cv::Mat_<double>(3, 1) << 1, cols - 0.5, -0.5);
	quads.push_back(cv::Point2f(pMat.at<double>(0), pMat.at<double>(1)));
}

void getDestinationFrameCorner(const cv::Mat& GridMatrix, std::vector<cv::Point2f>& quads, int rows, int cols)
{

}