/***************************************************************************//**
 *
 * @file JointControlCommandFsmFinger.h
 *
 * @brief This file contains the declaration of the JointControlCommandFsmFinger class.
 *
 ******************************************************************************/
#ifndef JOINTCONTROLCOMMANDFSMFINGER_H_
#define JOINTCONTROLCOMMANDFSMFINGER_H_

#include "nasa_robodyn_mechanisms_core/JointControlCommandFsmInterface.h"

class JointControlCommandFsmFinger : public JointControlCommandFsmInterface
{
public:
    JointControlCommandFsmFinger(const std::string&, IoFunctions);
    virtual ~JointControlCommandFsmFinger();

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
    std::string IsCalibratedLiveCoeffName;

private:
    std::string nameIsCalibrated;
};

#endif
