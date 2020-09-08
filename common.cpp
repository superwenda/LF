#include "common.hpp"
#include "MVCAPI.h"
#include <iostream>
#include <sstream>
#include <iomanip>
#include <functional>

MVCCamera::MVCCamera() : m_iDevNo(-1)
{
}

MVCCamera::~MVCCamera()
{
}

DWORD MVCCamera::Open(int card_no)
{
    if (m_iDevNo >= 0)
    {
        MVC_SetStreamHOOK(m_iDevNo, NULL, NULL);
    }
    m_iDevNo = card_no;
    using namespace std::placeholders;
    MVC_SetNetPacketSize(m_iDevNo, 1440);
    MVC_OpenDevice(m_iDevNo);
    return MVC_SetStreamHOOK(m_iDevNo, MVCCamera::StreamHook, this);
}
DWORD MVCCamera::Close()
{
    return MVC_CloseDevice(m_iDevNo);
}
DWORD MVCCamera::StartCapture()
{
    return MVC_EnableCapture(m_iDevNo);
}
DWORD MVCCamera::StopCapture()
{
    return MVC_DisableCapture(m_iDevNo);
}
DWORD MVCCamera::SetExposure(int exposure)
{
    return MVC_SetParameter(m_iDevNo, MVCADJ_INTTIME, exposure);
}
DWORD MVCCamera::SetGain(int gain)
{
    return MVC_SetParameter(m_iDevNo, MVCADJ_DACGAIN, gain);
}
DWORD MVCCamera::SetGamma(float gamma)
{
    return MVC_SetGammaValue(m_iDevNo, gamma, true);
}

void MVCCamera::PrintDevices()
{
    int n = MVC_GetDeviceNumber();
    MVCGE_DEVLISTEX DevInfoEx;
    for (int i = 0; i < n; ++i)
    {
        MVC_GetDeviceInfoEx(i, &DevInfoEx);
        std::stringstream ss;
        ss << i << ":";
        std::cout << std::setw(3) << ss.str() << DevInfoEx.DevIP << " " << DevInfoEx.DevMAC << "\n";
    }
}