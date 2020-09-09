#ifndef COMMON_INCLUDE_HPP
#define COMMON_INCLUDE_HPP

#include <afxwin.h>
#include <MVC_Common.h>

class MVCCamera
{
public:
    MVCCamera();
    virtual ~MVCCamera();

    void PrintDevices();

    DWORD Open(int card_no);
    DWORD Close();
    DWORD StartCapture();
    DWORD StopCapture();
    DWORD SetExposure(DWORD exposure);
    DWORD GetExposure(DWORD &exposure);
    DWORD SetGain(DWORD gain);
    DWORD GetGain(DWORD &gain);
    DWORD SetGamma(float gamma);

    DWORD AutoWhiteBalance();

    static UINT StreamHook(WORD wHWCardNo, MVCFRAMEINFO m_FrameInfo, PVOID pUserData)
    {
        return reinterpret_cast<MVCCamera *>(pUserData)->Process(m_FrameInfo);
    }
    virtual UINT Process(MVCFRAMEINFO &m_FrameInfo) = 0;

private:
    int m_iDevNo;
};

template <typename T>
class ThreadPool
{
public:
    ThreadPool() { CreateSemaphore(NULL, 0, 3, NULL); }
};

#endif //COMMON_INCLUDE_HPP