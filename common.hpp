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
    DWORD SetExposure(int exposure);
    DWORD SetGain(int gain);
    DWORD SetGamma(float gamma);

    static UINT StreamHook(WORD wHWCardNo, MVCFRAMEINFO m_FrameInfo, PVOID pUserData)
    {
        return reinterpret_cast<MVCCamera *>(pUserData)->Process(m_FrameInfo);
    }
    virtual UINT Process(MVCFRAMEINFO &m_FrameInfo) = 0;

private:
    int m_iDevNo;
};

#endif //COMMON_INCLUDE_HPP