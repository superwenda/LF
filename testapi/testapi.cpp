#include "common.hpp"
#include <string>
#include <iostream>
#include <sstream>
#include <memory>
#include <signal.h>

#include <opencv2/opencv.hpp>

std::thread::id MainID;

class Processor : public MVCCamera
{
public:
	Processor(){}
	virtual ~Processor() {}
	virtual UINT Process(MVCFRAMEINFO &m_FrameInfo)
	{

		cv::Mat frameRaw(cv::Size(m_FrameInfo.Width, m_FrameInfo.Height), CV_8UC1, m_FrameInfo.lBufPtr);
		cv::Mat frameBGR;
		// cv::cvtColor(frameRaw, frameBGR, cv::COLOR_BayerGB2BGR);
		// cv::pyrDown(frameBGR, frameBGR); cv::pyrDown(frameBGR, frameBGR);
		// std::stringstream ss;
		// ss << "Main ID: [" << MainID << "] Task ID: [" << std::this_thread::get_id() << "]";
		// cv::putText(frameBGR, ss.str().c_str(), cv::Point(20, 20), cv::FONT_HERSHEY_PLAIN, 1.0, cv::Scalar(0, 0, 255));
		// ss = std::stringstream();
		// ss << "SEQ: " << m_FrameInfo.FRAMEID;
		// cv::putText(frameBGR, ss.str().c_str(), cv::Point(20, 40), cv::FONT_HERSHEY_PLAIN, 1.0, cv::Scalar(0, 0, 255));
		// cv::imshow("Stream", frameBGR);
		// cv::waitKey(1);
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
	MainID = std::this_thread::get_id();
	signal(SIGINT, onSig);
	std::string cmd;
	std::shared_ptr<MVCCamera> pCamera(new Processor);
	pCamera->PrintDevices();
	std::cout << pCamera->Open(0) << "\n";
	std::cout << pCamera->StartCapture() << "\n";
	while (isOk)
		std::this_thread::sleep_for(std::chrono::seconds(5));
	
	return 0;
}