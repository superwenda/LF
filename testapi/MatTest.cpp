#include <opencv2/opencv.hpp>

int main()
{
    cv::Mat m = (cv::Mat_<double>(2, 3) << 1.3973238092677897e+02, 1.1707618135277446e+02,
                 -9.9003337471431418e-01, 8.6504848762065251e+01,
                 1.0003990125699507e+00, 1.1700081151719819e+02);

    std::cout << m.inv(cv::DecompTypes::DECOMP_SVD) << "\n";

    m = (cv::Mat_<double>(1,1)<<1.6);
    m.convertTo(m, CV_32S);
    std::cout << m <<"\n";
}