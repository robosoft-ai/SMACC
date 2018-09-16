/***************************************************************************//**
 *
 * @file JointControlCommandFsmInterface.h
 *
 * @brief This file contains the declaration of the JointControlCommandFsmInterface interface.
 *
 ******************************************************************************/
#ifndef JOINTCONTROLCOMMANDFSMINTERFACE_H_
#define JOINTCONTROLCOMMANDFSMINTERFACE_H_

#include <boost/shared_ptr.hpp>
#include <string>
#include "nasa_robodyn_mechanisms_core/JointControlCommonInterface.h"
#include <nasa_r2_common_msgs/JointControlData.h>

class JointControlCommandFsmInterface : public JointControlCommonInterface
{
public:
    virtual nasa_r2_common_msgs::JointControlData  getStates(void) = 0;

    // controlMode functions
    virtual void bootLoader(void) = 0;
    virtual void off(void)        = 0;
    virtual void park(void)       = 0;
    virtual void neutral(void)    = 0;
    virtual void drive(void)      = 0;

    // commandMode functions
    virtual void motCom(void)          = 0;
    virtual void stallMode(void)       = 0;
    virtual void multiLoopStep(void)   = 0;
    virtual void multiLoopSmooth(void) = 0;
    virtual void actuator(void)        = 0;

    // calibrationMode functions
    virtual void disableCalibrationMode(void) = 0;
    virtual void enableCalibrationMode(void)  = 0;

    // clearFaultMode functions
    virtual void enableClearFaultMode(void)  = 0;
    virtual void disableClearFaultMode(void) = 0;

protected:
    nasa_r2_common_msgs::JointControlData states;

    JointControlCommandFsmInterface(const std::string& mechanism, IoFunctions io)
        : JointControlCommonInterface(mechanism, io)
    {};
    virtual ~JointControlCommandFsmInterface() {};
    virtual void setParameters() = 0;
};

#endif
