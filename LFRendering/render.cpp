#include <opencv2/opencv.hpp>
#include "render.hpp"

namespace anakin
{

    Render::Render(cv::Mat &grid_matrix, int image_width, int image_height, double diameter, double ratio)
        : m_iWidth(image_width), m_iHeight(image_height), m_Diameter(diameter), m_Ratio(ratio)
    {
        m_matGridMatrix = grid_matrix;
        m_matGridMatrix.copyTo(m_matTransGridMatrix);
        m_matTransGridMatrix *= m_Ratio;

        std::cout << m_matTransGridMatrix << "\n";

        m_SrcCenters.clear();
        m_DstCenters.clear();
        for (int row = -2;; ++row)
        {
            int col;
            for (col = -1;; ++col)
            {
                cv::Mat m = (cv::Mat_<double>(1, 3) << col, row, 1) * m_matGridMatrix;
                cv::Point2d point((int)m.at<double>(0), (int)m.at<double>(1));
                cv::Rect ran(0, 0, m_iWidth, m_iHeight);
                if (point.inside(ran))
                {
                    m_SrcCenters.push_back(point);
                    m = (cv::Mat_<double>(1, 3) << col, row, 1) * m_matTransGridMatrix;
                    m_DstCenters.push_back(cv::Point2d((int)m.at<double>(0), (int)m.at<double>(1)));
                }
                else if (col >= 0)
                    break;
            }
            if (row > 0 && col == 0)
                break;
        }
        std::cout << "Quantity of anchor: " << m_SrcCenters.size() << "\n";
    }

    Render::~Render()
    {
    }

    cv::Mat Render::render(const cv::Mat &src_image, double zoom, double aperture, double hoffset, double voffset)
    {
        static cv::Mat dst_image = cv::Mat::zeros(m_iHeight * m_Ratio, m_iWidth * m_Ratio, src_image.type());
        static cv::Mat map_x = cv::Mat::zeros(m_iHeight * m_Ratio, m_iWidth * m_Ratio, CV_32FC1),
                map_y = cv::Mat::zeros(m_iHeight * m_Ratio, m_iWidth * m_Ratio, CV_32FC1);
        double aper_radius = m_Diameter * aperture * zoom / 2;
        cv::Point2d shift_center(aper_radius * (1. / aperture - 1) * hoffset, aper_radius * (1. / aperture - 1) * voffset);

        double render_range = m_Diameter * m_Ratio / sqrt(2);

        cv::Point2d rel_dst_point, dst_point, src_point;
        for (int r = -render_range; r <= render_range; ++r)
        {
            for (int c = -render_range; c <= render_range; ++c)
            {
                for (int n = 0; n < m_SrcCenters.size(); ++n)
                {
                    rel_dst_point.x = c;
                    rel_dst_point.y = r;
                    dst_point = m_DstCenters[n] + rel_dst_point;
                    src_point = m_SrcCenters[n] + rel_dst_point / zoom;
                    if (cv::norm(rel_dst_point / zoom - shift_center) > aper_radius ||
                        cv::norm(rel_dst_point) > render_range ||
                        !src_point.inside(cv::Rect(0, 0, m_iWidth, m_iHeight)) ||
                        !dst_point.inside(cv::Rect(0, 0, dst_image.cols, dst_image.rows)))
                    {
                        continue;
                    }
                    map_x.at<float>(cv::Point(dst_point)) = src_point.x;
                    map_y.at<float>(cv::Point(dst_point)) = src_point.y;
                }
            }
        }
        cv::remap(src_image, dst_image, map_x, map_y, cv::InterpolationFlags::INTER_LINEAR);
        return dst_image;
    }
    /*
    cv::Mat Render::render(const cv::Mat &src_image, double zoom, double aperture, double hoffset, double voffset)
    {
        static cv::Mat dst_image = cv::Mat::zeros(m_iHeight * m_Ratio, m_iWidth * m_Ratio, src_image.type());
        static cv::Mat map_x = cv::Mat::zeros(m_iHeight * m_Ratio, m_iWidth * m_Ratio, CV_32FC1),
                map_y = cv::Mat::zeros(m_iHeight * m_Ratio, m_iWidth * m_Ratio, CV_32FC1);
        double aper_radius = m_Diameter * aperture * zoom / 2;
        cv::Point2d shift_center(aper_radius * (1. / aperture - 1) * hoffset, aper_radius * (1. / aperture - 1) * voffset);

        cv::Point2d rel_dst_point, dst_point, src_point;
        for (int r = -aper_radius; r <= aper_radius; ++r)
        {
            for (int c = -aper_radius; c <= aper_radius; ++c)
            {
                for (int n = 0; n < m_SrcCenters.size(); ++n)
                {

                    rel_dst_point.x = c;
                    rel_dst_point.y = r;
                    dst_point = m_DstCenters[n] + shift_center + rel_dst_point;
                    src_point = m_SrcCenters[n] + (shift_center + rel_dst_point) / zoom;

                    if (cv::norm(rel_dst_point) > aper_radius ||
                        cv::norm(shift_center + rel_dst_point) > m_Diameter * m_Ratio / sqrt(2) ||
                        !src_point.inside(cv::Rect(0, 0, m_iWidth, m_iHeight)) ||
                        !dst_point.inside(cv::Rect(0, 0, dst_image.cols, dst_image.rows)))
                        continue;
                    map_x.at<float>(cv::Point(dst_point)) = src_point.x;
                    map_y.at<float>(cv::Point(dst_point)) = src_point.y;
                }
            }
        }

        cv::remap(src_image, dst_image, map_x, map_y, cv::InterpolationFlags::INTER_LINEAR);
        return dst_image;
    }
    */
} // namespace anakin