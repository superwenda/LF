#ifndef RENDER_INCLUDE_HPP
#define RENDER_INCLUDE_HPP

#include <opencv2/core/types.hpp>

namespace anakin
{
    class Render
    {
    private:
        cv::Mat m_matGridMatrix, m_matTransGridMatrix;
        int m_iWidth, m_iHeight;
        double m_Diameter;
        double m_Ratio;
        std::vector<cv::Point2d> m_SrcCenters, m_DstCenters;

    public:
        Render() = delete;
        Render(cv::Mat &grid_matrix, int image_width, int image_height, double diameter, double ratio);
        ~Render();

        cv::Mat render(const cv::Mat &src_image, double zoom, double aperture, double hoffset, double voffset);
    };

} // namespace anakin

#endif