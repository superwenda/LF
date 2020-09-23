#include <opencv2/opencv.hpp>
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

bool isInImage(cv::Point2d pt, int width, int height, double r)
{
    return r < pt.x && pt.x < width - r && r < pt.y && pt.y < height - r;
}
int main()
{
    // cv::FileStorage fs("D:/数据集/Lenslet2MVconNU/exe/calib.xml", cv::FileStorage::READ);
    double diameter(23.202295303345), rotation(1.570796370506), delta_rot(M_PI / 3.);
    double height, width, off_x(13.040943844572), off_y(-20.348051920159);
    cv::Point2d u(diameter * cos(rotation), diameter * sin(rotation)),
        v(diameter * cos(rotation + delta_rot), diameter * sin(rotation + delta_rot)),
        w(diameter * cos(rotation + 2 * delta_rot), diameter * sin(rotation + 2 *  delta_rot));
    std::cout << u << "\n"
              << v << "\n";

    cv::Mat src_img = cv::imread("D:/dataset/Lenslet2MVconNU/exe/Image/0000.png");

    height = src_img.rows;
    width = src_img.cols;
    cv::Point2d start_point(0.5 * (width - 1) + off_x, 0.5 * (height - 11) + off_y);
    int radius = 0, thickness = 1;
    cv::circle(src_img, start_point, radius, cv::Scalar(0, 0, 255), thickness);
    cv::Scalar sc[3] = {cv::Scalar(255, 0, 0), cv::Scalar(0, 255, 0), cv::Scalar(0, 0, 255)};
    for (int i = 0;; i++)
    {
        int j;
        for (j = 1;; j++)
        {
            cv::Point2d cur_point = start_point + u * i + v * j;
            if (!isInImage(cur_point, width, height, diameter / 2))
                break;
            cv::circle(src_img, cur_point, radius, sc[(i + j) % 3], thickness);
        }
        if (j == 1)
            break;
    }
    for (int i = 0;; i++)
    {
        int j;
        for (j = 1;; j++)
        {
            cv::Point2d cur_point = start_point + v * i + w * j;
            if (!isInImage(cur_point, width, height, diameter / 2))
                break;
            cv::circle(src_img, cur_point, radius, sc[(i + j) % 3], thickness);
        }
        if (j == 1)
            break;
    }
    for (int i = 0;; i++)
    {
        int j;
        for (j = 1;; j++)
        {
            cv::Point2d cur_point = start_point + w * i - u * j;
            if (!isInImage(cur_point, width, height, diameter / 2))
                break;
            cv::circle(src_img, cur_point, radius, sc[(i + j) % 3], thickness);
        }
        if (j == 1)
            break;
    }

    cv::imwrite("D:/dataset/Lenslet2MVconNU/exe/Image/0000_tag.bmp", src_img);
}