#ifndef CMAERA_GUI_INCLUDE_HPP
#define CMAERA_GUI_INCLUDE_HPP
#include "common.hpp"
class MVCCameraGUI : public MVCCamera
{
public:
    MVCCameraGUI();

    virtual ~MVCCameraGUI() {}

    void FlushCameraParam()
    {
        SetExposure(m_Exposure);
        SetGain(m_Gain);
        AutoWhiteBalance();
    }

private:
    int m_Exposure, m_Gain;
};

#endif