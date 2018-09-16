/***************************************************************************//**
 *
 * @file JointControlActualFsmFinger.h
 *
 * @brief This file contains the declaration of the JointControlActualFsmFinger class.
 *
 ******************************************************************************/
#ifndef JOINTCONTROLACTUALFSMFINGER_H_
#define JOINTCONTROLACTUALFSMFINGER_H_

#include "nasa_robodyn_mechanisms_core/JointControlActualFsmInterface.h"

class JointControlActualFsmFinger : public JointControlActualFsmInterface
{
public:
    JointControlActualFsmFinger(const std::string&, IoFunctions);
    virtual ~JointControlActualFsmFinger();

    nasa_r2_common_msgs::JointControlData getStates(void);
    void                                  updateControlModeState(void);
    void                                  updateCommandModeState(void);
    void                                  updateCalibrationModeState(void);
    void                                  updateClearFaultModeState(void);
    void                                  updateCoeffState(void);

protected:
    void setParameters();

    // Live coeffs names
    std::string IsCalibratedLiveCoeffName;
};

#endif
