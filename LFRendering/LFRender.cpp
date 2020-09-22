#include "camera_gui.hpp"
#include <opencv2/opencv.hpp>
#include <signal.h>
#include "render.hpp"

namespace chrono = std::chrono;

class Processor : public MVCCameraGUI
{
public:
    const int ZOOM_MAX = 300;
    const int SHIFT_LMT = 3;
    Processor(const std::string &calib_file) : m_Zoom(1.0), m_Aperture(1.0), tmpZoom(100), tmpAperture(5), tmpHOffset(SHIFT_LMT), tmpVOffset(SHIFT_LMT)
    {
        cv::createTrackbar("Zoom", "Parameter", &tmpZoom, ZOOM_MAX);
        cv::createTrackbar("Aperture", "Parameter", &tmpAperture, 10);
        cv::createTrackbar("HOffset", "Parameter", &tmpHOffset, SHIFT_LMT * 2);
        cv::createTrackbar("VOffset", "Parameter", &tmpVOffset, SHIFT_LMT * 2);

        cv::FileStorage fs(calib_file, cv::FileStorage::READ);
        cv::Mat grid_matrix;
        int width, height;
        double diameter;

        fs["ImageWidth"] >> width;
        fs["ImageHeight"] >> height;
        fs["Diameter"] >> diameter;
        fs["GridMatrix"] >> grid_matrix;
        m_pRender = std::shared_ptr<anakin::Render>(new anakin::Render(grid_matrix, width, height, diameter, 0.5));
    }
    virtual ~Processor()
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
        img_bgr = m_pRender->render(m_BGR, static_cast<double>(tmpZoom) / 100, tmpAperture / 10., (tmpHOffset - SHIFT_LMT) / static_cast<double>(SHIFT_LMT), (tmpVOffset - SHIFT_LMT) / static_cast<double>(SHIFT_LMT));
        int ratio = img_bgr.cols / 900;
        cv::Size sz(img_bgr.cols / ratio, img_bgr.rows / ratio);
        cv::resize(img_bgr, img_bgr, sz);
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

    std::shared_ptr<anakin::Render> m_pRender;
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
    Processor p("D:/workspace/LF/calibration.yaml");

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