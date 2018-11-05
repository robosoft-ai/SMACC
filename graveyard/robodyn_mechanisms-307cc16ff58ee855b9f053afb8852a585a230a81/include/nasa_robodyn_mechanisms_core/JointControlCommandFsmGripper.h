/***************************************************************************//**
 *
 * @file JointControlCommandFsmGripper.h
 *
 * @brief This file contains the declaration of the JointControlCommandFsmGripper class.
 *
 ******************************************************************************/
#ifndef JOINTCONTROLCOMMANDFSMGRIPPER_H_
#define JOINTCONTROLCOMMANDFSMGRIPPER_H_

#include "nasa_r2_config_core/NodeRegisterManager.h"
#include "nasa_robodyn_mechanisms_core/JointControlCommandFsmInterface.h"

class JointControlCommandFsmGripper : public JointControlCommandFsmInterface
{
public:
    JointControlCommandFsmGripper(const std::string&, IoFunctions, NodeRegisterManagerPtr);
    virtual ~JointControlCommandFsmGripper();

    nasa_r2_common_msgs::JointControlData getStates(void);

    // controlMode functions
    void bootLoader(void);
    void off(void);
    void park(void);
    void neutral(void);
    void drive(void);

    // commandMode functions
    void motCom(void);
    void stallMode(void);
    void multiLoopStep(void);
    void multiLoopSmooth(void);
    void actuator(void);

    // calibrationMode functions
    void disableCalibrationMode(void);
    void enableCalibrationMode(void);

    // clearFaultMode functions
    void enableClearFaultMode(void);
    void disableClearFaultMode(void);

protected:
    void setParameters();

    // Status bit names
    std::string CoeffsLoadedStatusName;

    // Control bit names
    std::string BootEnableControlName;
    std::string BridgeEnableControlName;
    std::string BrakeReleaseControlName;
    std::string MotorEnableControlName;
    std::string MotComSourceControlName;
    std::string ControlModeControlName;
    std::string CalibrationModeControlName;
    std::string ClearFaultControlName;

protected:
    NodeRegisterManagerPtr nodeRegisterManager;
};

#endif
