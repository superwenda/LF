#include "camera_gui.hpp"
#include <opencv2/opencv.hpp>

MVCCameraGUI::MVCCameraGUI() : m_Exposure(3431), m_Gain(100)
{
    cv::namedWindow("Parameter");
    cv::resizeWindow("Parameter", cv::Size(400, 600));
    
    cv::createTrackbar("Exposure", "Parameter", &m_Exposure, 3431);
    cv::createTrackbar("Gain", "Parameter", &m_Gain, 126);
}