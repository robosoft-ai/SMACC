/***************************************************************************//**
 *
 * @file JointControlActualFsmWrist.h
 *
 * @brief This file contains the declaration of the JointControlActualFsmWrist class.
 *
 ******************************************************************************/
#ifndef JOINTCONTROLACTUALFSMWRIST_H_
#define JOINTCONTROLACTUALFSMWRIST_H_

#include "nasa_robodyn_mechanisms_core/JointControlActualFsmInterface.h"

class JointControlActualFsmWrist : public JointControlActualFsmInterface
{
public:
    JointControlActualFsmWrist(const std::string&, IoFunctions);
    virtual ~JointControlActualFsmWrist();

    nasa_r2_common_msgs::JointControlData getStates(void);
    void                                  updateControlModeState(void);
    void                                  updateCommandModeState(void);
    void                                  updateCalibrationModeState(void);
    void                                  updateClearFaultModeState(void);
    void                                  updateCoeffState(void);

protected:
    void setParameters();

    // Live coeffs names
    std::string PitchLimitLiveCoeffName;
    std::string YawLimitLiveCoeffName;
    std::string LittlesideSliderLimitLiveCoeffName;
    std::string ThumbsideSliderLimitLiveCoeffName;
    std::string SensorErrorLiveCoeffName;
    std::string SliderDiffErrorLiveCoeffName;
    std::string ControlModeLiveCoeffName;
    std::string CommandModeLiveCoeffName;
    std::string CalibrationModeLiveCoeffName;
    std::string CoeffStateLiveCoeffName;
};

#endif
