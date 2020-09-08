#ifndef CALIB_GRID_INCLUDE_HPP
#define CALIB_GRID_INCLUDE_HPP
#include <vector>
#include <opencv2/opencv.hpp>

namespace wd
{
	void detectCenters(cv::Mat& binMat, std::vector<cv::Point2d>& centers);
	std::vector<cv::Point2d> cropBorder(std::vector<cv::Point2d> centers, size_t w, size_t h, size_t left, size_t right, size_t top, size_t bottom);
	template <typename T, class CMP>
	void makeHeap(T& vec, size_t size, size_t node, CMP& cmp);
	template <typename T, class CMP>
	void adjustHeap(T& vec, size_t size, size_t node, CMP& cmp);
	std::vector<std::vector<cv::Point2d>> detectGrid(std::vector<cv::Point2d> centers);
	double calcAngle(cv::Point2d& a, cv::Point2d& b);
	void detectRow(std::vector<cv::Point2d>& centers, std::vector<cv::Point2d>& row);
	void solvePoints(const std::vector<cv::Point2d>& pts, cv::Mat& line);
	void solveGrid(const std::vector<cv::Point2f>& points, size_t w, size_t h, cv::Mat& x, cv::Mat& y);
}

#endif // !CALIB_GRID_INCLUDE_HPP
