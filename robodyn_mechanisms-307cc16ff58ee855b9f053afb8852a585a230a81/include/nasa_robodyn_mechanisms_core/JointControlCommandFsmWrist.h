/***************************************************************************//**
 *
 * @file JointControlCommandFsmWrist.h
 *
 * @brief This file contains the declaration of the JointControlCommandFsmWrist class.
 *
 ******************************************************************************/
#ifndef JOINTCONTROLCOMMANDFSMWRIST_H_
#define JOINTCONTROLCOMMANDFSMWRIST_H_

#include "nasa_robodyn_mechanisms_core/JointControlCommandFsmInterface.h"

class JointControlCommandFsmWrist : public JointControlCommandFsmInterface
{
public:
    JointControlCommandFsmWrist(const std::string&, IoFunctions);
    virtual ~JointControlCommandFsmWrist();

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

    // Live coeffs names
    std::string ControlModeLiveCoeffName;
    std::string CommandModeLiveCoeffName;
    std::string CalibrationModeLiveCoeffName;

private:
    void updateControlMode();
    void updateCommandMode();
    void updateCalibrationMode();
};

#endif
