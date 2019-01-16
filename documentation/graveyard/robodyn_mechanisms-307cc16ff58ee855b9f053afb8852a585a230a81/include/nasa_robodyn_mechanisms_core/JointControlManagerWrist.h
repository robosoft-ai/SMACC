/***************************************************************************//**
 *
 * @file JointControlManagerWrist.h
 *
 * @brief This file contains the declaration of the JointControlManagerWrist class.
 *
 ******************************************************************************/
#ifndef JOINTCONTROLMANAGERWRIST_H_
#define JOINTCONTROLMANAGERWRIST_H_

#include "nasa_robodyn_mechanisms_core/JointControlManagerInterface.h"
#include "nasa_robodyn_mechanisms_core/JointControlActualFsmWrist.h"
#include "nasa_robodyn_mechanisms_core/JointControlCommandFsmWrist.h"

class JointControlManagerWrist : public JointControlManagerInterface
{
public:
    JointControlManagerWrist(const std::string&, IoFunctions, double, const std::string&);
    virtual ~JointControlManagerWrist();

    bool                                  verifyStates(void);
    bool                                  verifyControlModeState(void);
    bool                                  verifyCommandModeState(void);
    bool                                  verifyCalibrationModeState(void);
    bool                                  verifyClearFaultModeState(void);
    nasa_r2_common_msgs::JointControlData getActualStates(void);
    nasa_r2_common_msgs::JointControlData getCommandStates(void);
    void                                  setCommandStates(nasa_r2_common_msgs::JointControlData);
    std::string                           buildFaultString();

protected:
    void setParameters();

    // Live coeffs names
    std::string PitchLimitLiveCoeffName;
    std::string YawLimitLiveCoeffName;
    std::string LittlesideSliderLimitLiveCoeffName;
    std::string ThumbsideSliderLimitLiveCoeffName;
    std::string SensorErrorLiveCoeffName;
    std::string SliderDiffErrorLiveCoeffName;
};

#endif
