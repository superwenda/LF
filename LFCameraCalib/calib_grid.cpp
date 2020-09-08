#include "calib_grid.hpp"

void wd::detectCenters(cv::Mat& binMat, std::vector<cv::Point2d>& centers)
{
	std::vector<std::vector<cv::Point> > contours;
	std::vector<cv::Vec4i> hierachy;

	cv::findContours(binMat, contours, hierachy, cv::CHAIN_APPROX_SIMPLE, cv::RETR_TREE);
	int cnt = 0;
	size_t sz = contours.size();
	for (size_t i = 0; i < sz; ++i)
	{
		if (hierachy[i][3] >= 0)
			continue;
		cnt++;

		cv::Rect r = cv::boundingRect(contours[i]);
		cv::Scalar sc = cv::sum(contours[i]);
		cv::Point2d p(static_cast<double>(sc[0]) / contours[i].size(), static_cast<double>(sc[1]) / contours[i].size());

		centers.push_back(p);


	}
	std::cout << cnt << "\n";
}

std::vector<cv::Point2d> wd::cropBorder(std::vector<cv::Point2d> centers, size_t w, size_t h, size_t left, size_t right, size_t top, size_t bottom)
{
	for (auto i = centers.begin(); i != centers.end();)
	{
		if (i->x < left || i->x >= w - right || i->y < top || i->y >= h - bottom)
			i = centers.erase(i);
		else
			i++;

	}
	return centers;
}


template <typename T, class CMP>
void wd::adjustHeap(T& vec, size_t size, size_t node, CMP& cmp)
{
	if (size <= 1)
		return;
	size_t next = node, left = node * 2 + 1, right = node * 2 + 2;
	if (left < size && !cmp(vec[next], vec[left])) next = left;
	if (right < size && !cmp(vec[next], vec[right])) next = right;

	if (node != next)
	{
		std::swap(vec[node], vec[next]);
		adjustHeap(vec, size, next, cmp);
	}
}
template <typename T, class CMP>
void wd::makeHeap(T& vec, size_t size, size_t node, CMP& cmp)
{
	if (size <= 1)
		return;
	for (int i = (size - 2) / 2; ; --i)
	{
		adjustHeap(vec, size, i, cmp);
		if (i == 0)
			break;
	}
}


std::vector<std::vector<cv::Point2d>> wd::detectGrid(std::vector<cv::Point2d> centers)
{
	std::vector<std::vector<cv::Point2d>> grid;
	//std::vector<std::vector<cv::Point2d>> ret;

	while (!centers.empty())
	{
		makeHeap(centers, centers.size(), 0, [](cv::Point2d& a, cv::Point2d& b) {return (a.x + a.y) < (b.x + b.y); });
		std::vector<cv::Point2d> row;
		row.push_back(centers.front());
		std::swap(centers.front(), centers.back());
		centers.erase(centers.end() - 1);
		detectRow(centers, row);
		grid.push_back(row);
		std::cout << "Row size: " << row.size() << "\n";
	}
	return grid;
}

double wd::calcAngle(cv::Point2d& a, cv::Point2d& b)
{
	static const double M_PI = 3.14159265358979323846;
	cv::Point2d p = b - a;
	p = p / cv::norm(p);
	return std::acos(p.x) * 180 / M_PI;
}

void wd::detectRow(std::vector<cv::Point2d>& centers, std::vector<cv::Point2d>& row)
{
	if (row.size() != 1)
		throw std::exception();

	double angle_thresh = 15.0;

	while (!centers.empty())
	{
		cv::Point2d p;
		wd::makeHeap(centers, centers.size(), 0, [&](cv::Point2d& a, cv::Point2d& b) {return cv::norm(row.back() - a) < cv::norm(row.back() - b); });
		p = centers.front();
		if (centers.size() > 1)
		{
			std::swap(centers.front(), centers.back());
			centers.erase(centers.end() - 1);
			wd::adjustHeap(centers, centers.size(), 0, [&](cv::Point2d& a, cv::Point2d& b) {return cv::norm(row.back() - a) < cv::norm(row.back() - b); });
		}
		else
			centers.erase(centers.begin());

		if (calcAngle(row.back(), p) < angle_thresh)
		{
			row.push_back(p);
		}
		else if (!centers.empty() && calcAngle(row.back(), centers.front()) < angle_thresh)
		{
			row.push_back(centers.front());
			centers.front() = p;
		}
		else
		{
			centers.push_back(p);
			break;
		}
	}
}

void wd::solvePoints(const std::vector<cv::Point2d>& pts, cv::Mat& line)
{
	if (pts.size() < 2)
		throw std::exception("std::vector<cv::Point2d> solvePoints(std::vector<cv::Point2d>& pts):\n\tpts quantity not enough exception");

	cv::Mat A;
	for (auto& i : pts)
	{
		A.push_back(i.x);
		A.push_back(i.y);
		A.push_back(1.0);
	}
	A = A.reshape(1, pts.size());
	cv::SVD::solveZ(A, line);


}

void wd::solveGrid(const std::vector<cv::Point2f>& points, size_t w, size_t h, cv::Mat& x, cv::Mat& y)
{
	cv::Mat Ax, Bx, Ay, By;
	for (size_t i = 0; i < h; ++i)
	{
		for (size_t j = 0; j < w; ++j)
		{
			Ax.push_back(1.0);
			Ax.push_back(static_cast<double>(j));
			Ax.push_back(static_cast<double>(i));
			Bx.push_back(static_cast<double>(points[i * w + j].x));

			Ay.push_back(1.0);
			Ay.push_back(static_cast<double>(j));
			Ay.push_back(static_cast<double>(i));
			By.push_back(static_cast<double>(points[i * w + j].y));
		}
	}

	Ax = Ax.reshape(1, points.size());
	Bx = Bx.reshape(1, points.size());

	Ay = Ay.reshape(1, points.size());
	By = By.reshape(1, points.size());

	//cv::Mat a, b;
	cv::solve(Ax, Bx, x, cv::DECOMP_SVD);
	cv::solve(Ay, By, y, cv::DECOMP_SVD);
	//std::cout << x << "\n";
	//std::cout << y << "\n";
}