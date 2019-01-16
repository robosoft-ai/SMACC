/***************************************************************************//**
 *
 * @file JointControlManagerFinger.h
 *
 * @brief This file contains the declaration of the JointControlManagerFinger class.
 *
 ******************************************************************************/
#ifndef JOINTCONTROLMANAGERFINGER_H_
#define JOINTCONTROLMANAGERFINGER_H_

#include "nasa_robodyn_mechanisms_core/JointControlManagerInterface.h"
#include "nasa_robodyn_mechanisms_core/JointControlActualFsmFinger.h"
#include "nasa_robodyn_mechanisms_core/JointControlCommandFsmFinger.h"

class JointControlManagerFinger : public JointControlManagerInterface
{
public:
    JointControlManagerFinger(const std::string&, IoFunctions, double, const std::string&);
    virtual ~JointControlManagerFinger();

    bool                                  verifyStates(void);
    bool                                  verifyControlModeState(void);
    bool                                  verifyCommandModeState(void);
    bool                                  verifyCalibrationModeState(void);
    bool                                  verifyClearFaultModeState(void);
    nasa_r2_common_msgs::JointControlData getActualStates(void);
    nasa_r2_common_msgs::JointControlData getCommandStates(void);
    void                                  setCommandStates(nasa_r2_common_msgs::JointControlData);
    std::string                           buildFaultString();
};

#endif
