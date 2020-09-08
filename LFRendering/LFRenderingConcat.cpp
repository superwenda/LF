#include <opencv2/opencv.hpp>

class LFRend
{
public:
	LFRend() : GridCols(0), GridRows(0),
		GridMatrix((cv::Mat_<double>(2, 3) <<
			147, 118.07692308, 0.23809524,
			167, -0.38461538, 117.38095238)), 
		RFApertureSize(60)
	{

	}

	cv::Mat GetOddRF(const cv::Mat& src_img, double zoomrate, double aperture, double offsetx, double offsety)
	{
		cv::Mat ret = cv::Mat(RFApertureSize * GridRows, RFApertureSize * GridCols, src_img.type());

		std::vector<cv::Point2f> src_pts(4), dst_pts;
		dst_pts.push_back(cv::Point2f(0, 0));
		dst_pts.push_back(cv::Point2f(0, RFApertureSize - 1));
		dst_pts.push_back(cv::Point2f(RFApertureSize - 1, RFApertureSize - 1));
		dst_pts.push_back(cv::Point2f(RFApertureSize - 1, 0));

		for (int i = 0; i < GridRows; ++i)
		{
			for (int j = 0; j < GridCols; ++j)
			{
				if ((i + j) % 2 == 1)
				{
					cv::Mat tempmat;
					cv::Rect dstrect(j * RFApertureSize, i * RFApertureSize, RFApertureSize, RFApertureSize);
					tempmat = GridMatrix * (cv::Mat_<double>(3, 1) << 1, j - 0.5, i - 0.5);
					src_pts[0] = cv::Point2f(tempmat);
					tempmat = GridMatrix * (cv::Mat_<double>(3, 1) << 1, j - 0.5, i + 0.5);
					src_pts[0] = cv::Point2f(tempmat);
					tempmat = GridMatrix * (cv::Mat_<double>(3, 1) << 1, j + 0.5, i + 0.5);
					src_pts[0] = cv::Point2f(tempmat);
					tempmat = GridMatrix * (cv::Mat_<double>(3, 1) << 1, j + 0.5, i - 0.5);
					src_pts[0] = cv::Point2f(tempmat);

					tempmat = cv::getPerspectiveTransform(src_pts, dst_pts);
					cv::warpPerspective(src_img, ret(dstrect), tempmat, ret(dstrect).size());

				}
			}
		}
		return ret;
	}
	cv::Mat GetEveRF(const cv::Mat& src_img, double zoomrate, double aperture, double offsetx, double offsety)
	{

	}
	cv::Mat GridMatrix;
	int GridCols, GridRows;
	int RFApertureSize;
	//o(147,167) 13x(1682,162) 21y(152,2636) 35x25

	cv::Mat m_RFImage;
};

int main()
{

}