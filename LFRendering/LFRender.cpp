#include "camera_gui.hpp"
#include <opencv2/opencv.hpp>
#include <signal.h>

namespace chrono = std::chrono;

class Processor: public MVCCameraGUI
{
public:
    Processor(): m_Zoom(1.0), m_Aperture(1.0)
    {
        cv::createTrackbar("Zoom", "Parameter", &tmpZoom, 200);
        cv::createTrackbar("Aperture", "Parameter", &tmpAperture, 10);
        
    }
    ~Processor()
    {
        
    }
    virtual UINT Process(MVCFRAMEINFO &m_FrameInfo)
    {
        std::stringstream ss;
        ss << m_FrameInfo.FRAMEID;
        static cv::Mat img_gray, img_bgr;

        m_Raw = cv::Mat(cv::Size(m_FrameInfo.Width, m_FrameInfo.Height), CV_8UC1, m_FrameInfo.lBufPtr);

        m_Mutex.lock();
        cv::cvtColor(m_Raw, m_BGR, cv::COLOR_BayerGB2BGR);
        m_Mutex.unlock();

        int ratio = m_BGR.cols / 900;
        cv::Size sz(m_BGR.cols / ratio, m_BGR.rows / ratio);
        cv::resize(m_BGR, img_bgr, sz);
        cv::imshow("Raw", img_bgr);
        cv::waitKey(1);
        return 0ul;
    }

    void LoadCalibParam(const std::string &config)
    {
        cv::FileStorage fs(config, cv::FileStorage::READ);
        fs["ImageWidth"] >> m_CalibImageWidth;
        fs["ImageHeight"] >> m_CalibImageHeight;
        fs["GridMatrix"] >> m_GridMatrix;
        std::cout << "[" << m_CalibImageWidth << " x " << m_CalibImageHeight << "]\n";
        std::cout << m_GridMatrix << "\n";
    }

    

private:
    cv::Mat m_GridMatrix;
    int m_CalibImageWidth, m_CalibImageHeight;

    cv::Mat m_Raw, m_BGR, m_Gray, m_BinaryGray;
    std::mutex m_Mutex;

    double m_Zoom, m_Aperture, m_HOffset, m_VOffset;
    int tmpZoom, tmpAperture, tmpHOffset, tmpVOffset;
};

bool isOk = true;

void onSig(int sig)
{
    isOk = false;
}

int main()
{
    signal(SIGINT, onSig);
    std::string cmd;
    Processor p;
    p.LoadCalibParam("calibration.yaml");

    p.PrintDevices();
    p.Open(0);
    p.StartCapture();
    char key;
    while (isOk)
    {
        key = cv::waitKey(10);
        if (key == 27)
            break;
        else if (key == 'f')
        {
            p.FlushCameraParam();
        }
    }
    p.StopCapture();
    p.Close();
}