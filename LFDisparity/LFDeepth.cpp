#include <opencv2/opencv.hpp>

void getSourcePoints(const cv::Mat& GridMatrix, int row, int col, std::vector<cv::Point2f>& ret)
{
	double dx[4]{ -0.5, -0.5, 0.5, 0.5 }, dy[4]{ -0.5, 0.5, 0.5, -0.5 };

	for (int i = 0; i < 4; ++i)
	{
		cv::Mat v = (cv::Mat_<double>(3, 1) << 1, col + dx[i], row + dy[i]);
		cv::Mat r = (GridMatrix * v);
		ret.push_back(cv::Point2f(r.at<double>(0), r.at<double>(1)));
	}
}

void getDestinationPoints(const cv::Mat& GridMatrix, int row, int col, std::vector<cv::Point2f>& ret, double zoom)
{
	double dx[4]{ -0.8, -0.8, 0.8, 0.8 }, dy[4]{ -0.8, 0.8, 0.8, -0.8 };

	for (int i = 0; i < 4; ++i)
	{
		cv::Mat v = (cv::Mat_<double>(3, 1) << 0, 0.4 * col + dx[i] * zoom, 0.4 * row + dy[i] * zoom);
		cv::Mat r = (GridMatrix * v);
		ret.push_back(cv::Point2f(r.at<double>(0), r.at<double>(1)));
	}
}

void transformBlock(const cv::Mat& src_img, cv::Mat& dst_img, std::vector<cv::Point2f> from, std::vector<cv::Point2f> to, const cv::Mat& aperture)
{
	if (src_img.empty())
		throw std::exception("src_img empty!");
	cv::Rect rect = cv::boundingRect(to);
	cv::Mat block = cv::Mat::zeros(rect.height, rect.width, src_img.type()), block_ = cv::Mat::zeros(rect.height, rect.width, src_img.type());



	std::vector<cv::Point2f> tmp_to;
	for (auto& i : to)
	{
		tmp_to.push_back(i - static_cast<cv::Point2f>(rect.tl()));
	}
	cv::Mat transMat = cv::getPerspectiveTransform(from, tmp_to);
	cv::warpPerspective(src_img, block, transMat, block.size());
	block &= aperture;
	int shiftX(0), shiftY(0);
	if (rect.x < 0)
		shiftX = rect.x;
	else if (rect.x + rect.width > dst_img.cols)
		shiftX = (dst_img.cols - rect.x - rect.width);
	if (rect.y < 0)
		shiftY = rect.y;
	else if (rect.y + rect.height > dst_img.rows)
		shiftY = (dst_img.rows - rect.y - rect.height);

	cv::Mat M = (cv::Mat_<double>(2, 3) << 1, 0, shiftX, 0, 1, shiftY);
	cv::warpAffine(block, block_, M, block.size(), 1, cv::BORDER_CONSTANT, cv::Scalar(0, 0, 0));
	rect.x = std::max(rect.x, 0);
	rect.x = std::min(rect.x, dst_img.cols - rect.width);
	rect.y = std::max(rect.y, 0);
	rect.y = std::min(rect.y, dst_img.rows - rect.height);

	dst_img(rect) += block_;
}

void transformBlockMask(cv::Mat& dst_img, std::vector<cv::Point2f> from, std::vector<cv::Point2f> to, const cv::Mat& aperture)
{

	cv::Rect rect = cv::boundingRect(to);
	cv::Mat aperture_ = cv::Mat::zeros(rect.size(), CV_8UC1);
	//cv::Mat aperture = cv::getStructuringElement(cv::MORPH_ELLIPSE, rect.size()), aperture_=cv::Mat::zeros(rect.size(), CV_8UC1);
	//cv::Size sz = aperture.size();
	//sz.width *= (1. - apertureSize);
	//sz.height *= (1. - apertureSize);
	//sz.width += (1 - sz.width % 2);
	//sz.height += (1 - sz.height % 2);
	//cv::erode(aperture, aperture, cv::getStructuringElement(cv::MORPH_ELLIPSE, sz), cv::Point(-1, -1), 1, 0, cv::Scalar(0, 0, 0));
	//cv::cvtColor(aperture, aperture, cv::COLOR_GRAY2BGR);

	int shiftX(0), shiftY(0);
	if (rect.x < 0)
		shiftX = rect.x;
	else if (rect.x + rect.width > dst_img.cols)
		shiftX = (dst_img.cols - rect.x - rect.width);
	if (rect.y < 0)
		shiftY = rect.y;
	else if (rect.y + rect.height > dst_img.rows)
		shiftY = (dst_img.rows - rect.y - rect.height);

	cv::Mat M = (cv::Mat_<double>(2, 3) << 1, 0, shiftX, 0, 1, shiftY);
	cv::warpAffine(aperture, aperture_, M, aperture.size(), 1, cv::BORDER_CONSTANT, cv::Scalar(0, 0, 0));
	rect.x = std::max(rect.x, 0);
	rect.x = std::min(rect.x, dst_img.cols - rect.width);
	rect.y = std::max(rect.y, 0);
	rect.y = std::min(rect.y, dst_img.rows - rect.height);

	cv::cvtColor(aperture_, aperture_, cv::COLOR_BGR2GRAY);
	dst_img(rect) += aperture_;
}

void getAperture(cv::Mat& aperture, const std::vector<cv::Point2f>& to, double apertureSize, double offsetX, double offsetY)
{

	cv::Rect rect = cv::boundingRect(to);

	aperture = cv::getStructuringElement(cv::MORPH_ELLIPSE, rect.size());
	cv::Size sz = rect.size();
	sz.width *= (1. - apertureSize);
	sz.height *= (1. - apertureSize);
	sz.width += (1 - sz.width % 2);
	sz.height += (1 - sz.height % 2);
	cv::erode(aperture, aperture, cv::getStructuringElement(cv::MORPH_ELLIPSE, sz), cv::Point(-1, -1), 1, 0, cv::Scalar(0, 0, 0));

	cv::Mat M = (cv::Mat_<double>(2, 3) << 1, 0, offsetX / 2. * sz.width, 0, 1, offsetY / 2. * sz.height);
	cv::warpAffine(aperture, aperture, M, aperture.size(), 1, 0, cv::Scalar(0));

	cv::cvtColor(aperture, aperture, cv::COLOR_GRAY2BGR);
}

void LFRefocus(const cv::Mat& src_img, cv::Mat& dst_img, const cv::Mat& GridMatrix, int rows, int cols, double zoom, double apertureSize, double offsetX, double offsetY)
{

	dst_img = cv::Mat::zeros(src_img.rows * 0.4, src_img.cols * 0.4, CV_32FC3);
	cv::Mat aperture;
	cv::Mat dst_img_mask;
	dst_img_mask = cv::Mat::zeros(src_img.rows * 0.4, src_img.cols * 0.4, CV_8U);
	for (int i = 0; i < rows; ++i)
	{
		for (int j = 0; j < cols; ++j)
		{
			//std::cout << "RF" << i << "\t" << j << "\n";

			std::vector<cv::Point2f> from, to;
			getSourcePoints(GridMatrix, i, j, from);
			getDestinationPoints(GridMatrix, i, j, to, zoom);
			cv::Rect rect = cv::boundingRect(to);
			if (((i == 0) && (j == 0)) || (rect.size() != aperture.size()))
			{
				getAperture(aperture, to, apertureSize, offsetX, offsetY);
			}
			transformBlock(src_img, dst_img, from, to, aperture * 255);
			transformBlockMask(dst_img_mask, from, to, aperture);
		}
	}
	cv::imshow("Block Mask", aperture * 255);
	dst_img_mask += ((dst_img_mask == 0) / 255);
	cv::cvtColor(dst_img_mask, dst_img_mask, cv::COLOR_GRAY2BGR);
	//std::cout << dst_img_mask << "\n";
	cv::imshow("Mask", dst_img_mask * 10);
	dst_img_mask.convertTo(dst_img_mask, CV_32FC3);
	dst_img /= dst_img_mask;
	dst_img.convertTo(dst_img, CV_8UC3);
}

int main(int argc, char** argv)
{

}