/***************************************************************************//**
 *
 * @file JointControlActualFsmGripper.h
 *
 * @brief This file contains the declaration of the JointControlActualFsmGripper class.
 *
 ******************************************************************************/
#ifndef JOINTCONTROLACTUALFSMGRIPPER_H_
#define JOINTCONTROLACTUALFSMGRIPPER_H_

#include "nasa_robodyn_mechanisms_core/JointControlActualFsmInterface.h"
#include "nasa_r2_config_core/NodeRegisterManager.h"

class JointControlActualFsmGripper : public JointControlActualFsmInterface
{
public:
    JointControlActualFsmGripper(const std::string&, IoFunctions, NodeRegisterManagerPtr);
    virtual ~JointControlActualFsmGripper();

    nasa_r2_common_msgs::JointControlData getStates(void);
    void                                  updateControlModeState(void);
    void                                  updateCommandModeState(void);
    void                                  updateCalibrationModeState(void);
    void                                  updateClearFaultModeState(void);
    void                                  updateCoeffState(void);

protected:
    void setParameters();

    // Status bit names
    std::string ProcAliveStatusName;
    std::string CommAliveStatusName;
    std::string JointFaultStatusName;
    std::string CoeffsLoadedStatusName;
    std::string MotorEnableStatusName;
    std::string BrakeReleaseStatusName;
    std::string BridgeEnableStatusName;
    std::string MotComSourceStatusName;
    std::string ControlModeStatusName;
    std::string CalibrationModeStatusName;
    std::string ClearFaultStatusName;

    // Control bit names
    std::string BootEnableControlName;

protected:
    NodeRegisterManagerPtr nodeRegisterManager;
};

#endif
