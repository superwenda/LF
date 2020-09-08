#include "common.hpp"
#include <string>
#include <iostream>
#include <memory>
#include <signal.h>

#include <opencv2/opencv.hpp>

class Processor : public MVCCamera
{
public:
	Processor(){}
	virtual ~Processor() {}
	virtual UINT Process(MVCFRAMEINFO &m_FrameInfo)
	{
		std::cout << std::setw(10) << std::left << m_FrameInfo.FRAMEID << std::setw(4) << ": [" << m_FrameInfo.Width << ", " << m_FrameInfo.Height << "] " << m_FrameInfo.lBufSize << "\n";
		cv::Mat frameRaw(cv::Size(m_FrameInfo.Width, m_FrameInfo.Height), CV_8UC1, m_FrameInfo.lBufPtr);
		cv::Mat frameBGR;
		cv::cvtColor(frameRaw, frameBGR, cv::COLOR_BayerGB2BGR);
		cv::pyrDown(frameBGR, frameBGR);
		cv::pyrDown(frameBGR, frameBGR);
		cv::imshow("Stream", frameBGR);
		cv::waitKey(1);
		return 0u;
	}
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
	while (isOk)
	{
		/* code */
	}
	
	return 0;
}