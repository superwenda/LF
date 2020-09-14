#include <opencv2/opencv.hpp>

int main()
{
    cv::FileStorage fs("calibration.yaml", cv::FileStorage::READ);
    int width, height;
    double diameter;
    cv::Mat GridMatrix;

    fs["ImageWidth"] >> width;
    fs["ImageHeight"] >> height;
    fs["Diameter"] >> diameter;
    fs["GridMatrix"] >> GridMatrix;

    cv::Mat raw = cv::Mat::zeros(height, width, CV_8UC3);
    cv::Mat rend = cv::Mat::zeros(height / 2, width / 2, CV_8UC3);
    for (int row = -2;; ++row)
    {
        int col;
        for (col = -1;; ++col)
        {
            cv::Mat m = (cv::Mat_<double>(1, 3) << col, row, 1) * GridMatrix;
            cv::Point point((int)m.at<double>(0), (int)m.at<double>(1));
            // cv::Rect ran(cv::Point(left, top), cv::Point(m_Gray.cols - right, m_Gray.rows - bottom));
            cv::Rect ran(0, 0, raw.cols, raw.rows);
            if (point.inside(ran))
            {
                cv::circle(raw, point, diameter / 2, cv::Scalar(255, 255, 0), 1);
            }
            else if (col >= 0)
                break;
        }
        if (row > 0 && col == 0)
            break;
    }

    cv::Mat rGridMatrix;
    GridMatrix.copyTo(rGridMatrix);
    rGridMatrix.rowRange(0, 2) /= 2;
    for (int row = -2;; ++row)
    {
        int col;
        for (col = -1;; ++col)
        {
            cv::Mat m = (cv::Mat_<double>(1, 3) << col, row, 1) * rGridMatrix;
            cv::Point point((int)m.at<double>(0), (int)m.at<double>(1));
            // cv::Rect ran(cv::Point(left, top), cv::Point(m_Gray.cols - right, m_Gray.rows - bottom));
            cv::Rect ran(0, 0, rend.cols, rend.rows);
            if (point.inside(ran))
            {
                cv::circle(rend, point, diameter / 2., cv::Scalar(255, 255, 0), 1);
            }
            else if (col >= 0)
                break;
        }
        if (row > 0 && col == 0)
            break;
    }
    // int ratio = raw.cols / 900;
    // cv::Size sz(raw.cols / ratio, raw.rows / ratio);
    // cv::resize(raw, raw, sz);

    // ratio = rend.cols / 600;
    // sz = cv::Size(rend.cols / ratio, rend.rows / ratio);
    // cv::resize(rend, rend, sz);

    cv::imshow("Raw", raw);
    cv::imshow("rend", rend);
    cv::waitKey();
}