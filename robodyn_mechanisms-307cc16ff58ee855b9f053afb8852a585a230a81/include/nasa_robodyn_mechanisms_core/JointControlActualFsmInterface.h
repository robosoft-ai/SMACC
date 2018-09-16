/***************************************************************************//**
 *
 * @file JointControlActualFsmInterface.h
 *
 * @brief This file contains the declaration of the JointControlActualFsmInterface interface.
 *
 ******************************************************************************/
#ifndef JOINTCONTROLACTUALFSMINTERFACE_H_
#define JOINTCONTROLACTUALFSMINTERFACE_H_

#include <boost/shared_ptr.hpp>
#include <string>
#include "nasa_robodyn_mechanisms_core/JointControlCommonInterface.h"
#include <nasa_r2_common_msgs/JointControlData.h>

class JointControlActualFsmInterface : public JointControlCommonInterface
{
public:
    virtual nasa_r2_common_msgs::JointControlData getStates(void)                  = 0;
    virtual void                                  updateControlModeState(void)     = 0;
    virtual void                                  updateCommandModeState(void)     = 0;
    virtual void                                  updateCalibrationModeState(void) = 0;
    virtual void                                  updateClearFaultModeState(void)  = 0;
    virtual void                                  updateCoeffState(void)           = 0;
 
protected:
    JointControlActualFsmInterface(const std::string& mechanism, IoFunctions io)
        : JointControlCommonInterface(mechanism, io)
    {};
    virtual ~JointControlActualFsmInterface() {};
    virtual void setParameters() = 0;

    nasa_r2_common_msgs::JointControlData states;
};

#endif
