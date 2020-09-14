#include "render.hpp"
#include <opencv2/opencv.hpp>

namespace anakin
{

    Render::Render(cv::Mat &GridMatrix, int ImageWidth, int ImageHeight, double Ratio) : m_matGridMatrix(GridMatrix), m_iWidth(ImageWidth), m_iHeight(ImageHeight), m_Ratio(Ratio)
    {
    }

    Render::~Render()
    {
    }
} // namespace anakin