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
        double m_Ratio;
    public:
        Render(cv::Mat &GridMatrix, int ImageWidth, int ImageHeight, double Ratio);
        ~Render();

        cv::Mat render(const cv::Mat &srcImage, double zoom, double aperture, double hoffset, double voffset);
    };

} // namespace anakin

#endif