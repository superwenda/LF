#include <afxwin.h>
#include <iostream>
#include <iomanip>
#include <signal.h>
#include <chrono>
#include <thread>

#include "MVCAPI.h"
#include <opencv2/opencv.hpp>

int isOk = 1;

void onSignal(int sig)
{
    isOk = 0;
}

int main(int argc, char **argv)
{
    signal(SIGINT, onSignal);
    int devNo(0);
    MVC_ReScanDevice();
    int devQty = MVC_GetDeviceNumber();
    std::cout << devQty << "\n";
    MVCGE_DEVLISTEX DevInfo;
    for (int i = 0; i < devQty; ++i)
    {
        MVC_GetDeviceInfoEx(i, &DevInfo);
        std::cout << std::setw(3) << std::left << i << ": " << std::setw(20) << std::left << DevInfo.DevIP << std::setw(20) << std::right << DevInfo.DevMAC << "\n";
    }
    MVC_AutoIPConfig(devNo);
    int ret = 0, cnt = 5;
    do
    {
        ret = MVC_OpenDevice(devNo);
        std::cout << "Open device with return: " << ret << "\n";
        cnt--;
        // std::this_thread::sleep_for(std::chrono::milliseconds(500));
    } while (ret != 0 && cnt >= 0);

    if (ret != 0)
    {
        std::cerr << "Open device fail\n";
        exit(1);
    }

    // MVC_SetParameter(devNo, MVCDISP_ISDISP, 0);
    // MVC_AutoWhiteBalance(0);
    for (int i = 0; i < 5; ++i)
        MVC_SetParameter(devNo, MVCADJ_DACGAIN, 100);

    double gain = MVC_GetParameter(0, MVCADJ_DACGAIN), exposure = MVC_GetParameter(0, MVCADJ_INTTIME);
    std::cout << "Init gain: " << gain << ", exposure: " << exposure << "\n";
    std::cout << "Enable capture: " << MVC_EnableCapture(devNo) << "\n";

    MVCFRAMEINFO Frame;
    std::chrono::time_point<std::chrono::steady_clock> time_stamp = std::chrono::steady_clock::now();
    cv::Mat frameRaw(3288, 4384, CV_8UC1), frameBGR(3288, 4384, CV_8UC3);
    while (isOk)
    {
        

        // std::cout << "Grab image\n";
        memset(&Frame, 0, sizeof(MVCFRAMEINFO));
        MVC_GetRawData(devNo, &Frame);
        // std::cout << std::setw(10) << std::left << Frame.FRAMEID << std::setw(4) << ": [" << Frame.Width << ", " << Frame.Height << "] " << Frame.lBufSize << "\n";
        // std::cout << "Pixel type: " << Frame.PixelID << "\n";
        // time_stamp = std::chrono::steady_clock::now();
        // std::cout << "At " << std::chrono::duration_cast<std::chrono::milliseconds>(time_stamp - std::chrono::time_point<std::chrono::steady_clock>()).count() << " millisecond \n";

        // memcpy(frameRaw.data, Frame.lBufPtr, Frame.lBufSize);
        frameRaw = cv::Mat(cv::Size(Frame.Width, Frame.Height), CV_8UC1, Frame.lBufPtr);
        if(frameRaw.empty())
            continue;
        cv::cvtColor(frameRaw, frameBGR, cv::COLOR_BayerGB2BGR);
        cv::pyrDown(frameBGR, frameBGR);
        cv::pyrDown(frameBGR, frameBGR);
        cv::imshow("Stream", frameBGR);
        cv::waitKey(1);

    }
    MVC_DisableCapture(devNo);
    MVC_CloseDevice(devNo);
    std::cout << "Unregister success\n";
    return 0;
}