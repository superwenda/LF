#include "camera_gui.hpp"
#include <string>
#include <iostream>
#include <sstream>
#include <memory>
#include <signal.h>

#include <opencv2/opencv.hpp>

#include "calibration.hpp"

namespace chrono = std::chrono;
class Processor : public MVCCameraGUI
{
public:
    Processor() : left(70), right(70), top(60), bottom(90), medianBlurSize(7), blockSize(31), C(1), openOpBlockSize(13) //, m_Exposure(3431), m_Gain(100)
    {
        cv::namedWindow("Process");
        cv::createTrackbar("medianBlurSize", "Parameter", &medianBlurSize, 60);
        cv::createTrackbar("blockSize", "Parameter", &blockSize, 60);
        cv::createTrackbar("C", "Parameter", &C, 60);
        cv::createTrackbar("enrodBlockSize", "Parameter", &openOpBlockSize, 60);

        cv::createTrackbar("left", "Parameter", &left, 100);
        cv::createTrackbar("right", "Parameter", &right, 100);
        cv::createTrackbar("top", "Parameter", &top, 100);
        cv::createTrackbar("bottom", "Parameter", &bottom, 100);

    }
    virtual ~Processor() {}
    virtual UINT Process(MVCFRAMEINFO &m_FrameInfo)
    {
        std::stringstream ss;
        ss << m_FrameInfo.FRAMEID;
        static cv::Mat img_gray, img_bgr;
        chrono::time_point<chrono::steady_clock> tp_begin = chrono::steady_clock::now();
        m_Raw = cv::Mat(cv::Size(m_FrameInfo.Width, m_FrameInfo.Height), CV_8UC1, m_FrameInfo.lBufPtr);

        m_Mutex.lock();
        cv::cvtColor(m_Raw, m_BGR, cv::COLOR_BayerGB2BGR);
        m_Mutex.unlock();

        int ratio = m_BGR.cols / 800;
        cv::Size sz(m_BGR.cols / ratio, m_BGR.rows / ratio);
        cv::resize(m_BGR, img_bgr, sz);
        cv::imshow("Raw", img_bgr);
        cv::waitKey(1);

        return 0u;
    }

    void ShowBinary()
    {
        if (m_BGR.empty())
        {
            std::cerr << __FILE__ << "(" << __LINE__ << "): m_BGR.empty()==true\n";
            return;
        }
        static cv::Mat viewImage;
        m_Mutex.lock();
        cv::cvtColor(m_BGR, m_Gray, cv::COLOR_BGR2GRAY);
        m_Mutex.unlock();

        GetBinaryImage(m_Gray, m_BinaryGray);
        int ratio = m_BinaryGray.cols / 800;
        cv::Size sz(m_BinaryGray.cols / ratio, m_BinaryGray.rows / ratio);
        cv::resize(m_BinaryGray, viewImage, sz);
        cv::imshow("Process", viewImage);
    }

    void DrawCenters()
    {
        if (m_Gray.empty())
        {
            std::cerr << __FILE__ << "(" << __LINE__ << "): m_Gray.empty()==true\n";
            return;
        }
        static cv::Mat viewImage;
        m_Centers.clear();
        anakin::DetectCenters(m_BinaryGray, m_Centers);
        m_Centers = anakin::CropBorder(m_Centers, m_BinaryGray.cols, m_BinaryGray.rows, left, right, top, bottom);
        m_Mutex.lock();
        m_BGR.copyTo(viewImage);
        m_Mutex.unlock();

        for (auto &p : m_Centers)
        {
            cv::circle(viewImage, p, 10, cv::Scalar(0, 0, 255), 7);
        }
        int ratio = viewImage.cols / 800;
        cv::Size sz(viewImage.cols / ratio, viewImage.rows / ratio);
        cv::resize(viewImage, viewImage, sz);
        cv::imshow("Process", viewImage);
    }

    void CalibGridMatrix()
    {
        static cv::Mat viewImage;
        m_Mutex.lock();
        m_BGR.copyTo(viewImage);
        m_Mutex.unlock();
        double est_dist = anakin::EstimateAverageDist(m_Centers);
        m_GridMatrix = anakin::DetectGridMatrix(m_Centers, est_dist);
        std::cout << "Grid Matrix: " << m_GridMatrix << "\n";
        for (int row = -2;; ++row)
        {
            int col;
            for (col = -1;; ++col)
            {
                cv::Mat m = (cv::Mat_<double>(1, 3) << col, row, 1) * m_GridMatrix;
                cv::Point point((int)m.at<double>(0), (int)m.at<double>(1));
                cv::Rect ran(cv::Point(left, top), cv::Point(m_Gray.cols - right, m_Gray.rows - bottom));
                // cv::Rect ran(0, 0, m_Gray.cols, m_Gray.rows);
                if (point.inside(ran))
                {
                    cv::circle(viewImage, point, est_dist / 2, cv::Scalar(255, 255, 0), 5);
                }
                else if (col >= 0)
                    break;
            }
            if (row > 0 && col == 0)
                break;
        }
        int ratio = viewImage.cols / 800;
        cv::Size sz(viewImage.cols / ratio, viewImage.rows / ratio);
        cv::resize(viewImage, viewImage, sz);
        cv::imshow("Process", viewImage);
    }

    void SaveGridMatrix()
    {
        cv::FileStorage fs("calibration.yaml", cv::FileStorage::WRITE);
        
        fs << "ImageWidth" << m_BGR.cols;
        fs << "ImageHeight" << m_BGR.rows;
        fs << "GridMatrix" << m_GridMatrix;
        
    }

private:
    void GetBinaryImage(cv::Mat &src_gray_img, cv::Mat &dst_binary_img)
    {
        if (src_gray_img.channels() != 1)
        {
            std::cerr << __FILE__ << "(" << __LINE__ << "): src_gray_img.channels() != 1\n";
            return;
        }
        cv::medianBlur(src_gray_img, dst_binary_img, medianBlurSize * 2 + 1);
        cv::adaptiveThreshold(dst_binary_img, dst_binary_img, 255, cv::ADAPTIVE_THRESH_GAUSSIAN_C, cv::THRESH_BINARY, blockSize * 2 + 3, static_cast<double>(C) / 100.0);
        cv::morphologyEx(dst_binary_img, dst_binary_img, cv::MORPH_OPEN, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(openOpBlockSize * 2 + 1, openOpBlockSize * 2 + 1)));
    }

    cv::Mat m_Raw, m_BGR, m_Gray, m_BinaryGray;
    std::vector<cv::Point2d> m_Centers;
    cv::Mat m_GridMatrix;

    int medianBlurSize, blockSize, C, openOpBlockSize;
    int left, right, top, bottom;
    std::mutex m_Mutex;

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
    std::shared_ptr<MVCCamera> pCamera(new Processor);
    pCamera->PrintDevices();
    std::cout << pCamera->Open(0) << "\n";
    std::cout << pCamera->StartCapture() << "\n";
    char key;
    while (isOk)
    {
        // std::this_thread::sleep_for(std::chrono::seconds(5));
        key = cv::waitKey(1);

        if (key == 27)
            break;
        else if (key == '1')
        {
            dynamic_cast<Processor *>(&*pCamera)->ShowBinary();
        }
        else if (key == '2')
        {
            dynamic_cast<Processor *>(&*pCamera)->DrawCenters();
        }
        else if (key == '3')
        {
            dynamic_cast<Processor *>(&*pCamera)->CalibGridMatrix();
        }
        else if (key == 's')
        {
            dynamic_cast<Processor *>(&*pCamera)->SaveGridMatrix();
        }
        else if (key == 'f')
        {
            dynamic_cast<Processor *>(&*pCamera)->FlushCameraParam();
        }
        
    }

    pCamera->StopCapture();
    pCamera->Close();
    return 0;
}