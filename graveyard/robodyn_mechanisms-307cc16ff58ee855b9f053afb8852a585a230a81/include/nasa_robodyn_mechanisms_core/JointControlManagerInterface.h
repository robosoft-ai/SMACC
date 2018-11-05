/***************************************************************************//**
 *
 * @file JointControlManagerInterface.h
 *
 * @brief This file contains the declaration of the JointControlManagerInterface interface.
 *
 ******************************************************************************/
#ifndef JOINTCONTROLMANAGERINTERFACE_H_
#define JOINTCONTROLMANAGERINTERFACE_H_

#include <boost/shared_ptr.hpp>
#include <string>
#include <map>
#include "nasa_robodyn_mechanisms_core/JointControlActualFsmInterface.h"
#include "nasa_robodyn_mechanisms_core/JointControlCommandFsmInterface.h"

class JointControlManagerInterface : public JointControlCommonInterface
{
public:
    virtual bool                                  verifyStates(void)                                      = 0;
    virtual bool                                  verifyControlModeState(void)                            = 0;
    virtual bool                                  verifyCommandModeState(void)                            = 0;
    virtual bool                                  verifyCalibrationModeState(void)                        = 0;
    virtual bool                                  verifyClearFaultModeState(void)                         = 0;
    virtual nasa_r2_common_msgs::JointControlData getActualStates(void)                                   = 0;
    virtual nasa_r2_common_msgs::JointControlData getCommandStates(void)                                  = 0;
    virtual void                                  setCommandStates(nasa_r2_common_msgs::JointControlData) = 0;
    virtual std::string                           buildFaultString()                                      = 0;

    double getTimeLimit(void)
    {
        return timeLimit;
    }

    void setTimeLimit(double newTimeLimit)
    {
        timeLimit = newTimeLimit;
    }

    std::string getType(void)
    {
        return type;
    }

protected:
    nasa_r2_common_msgs::JointControlData              actualStates;
    nasa_r2_common_msgs::JointControlData              commandStates;
    nasa_r2_common_msgs::JointControlData              prevActualStates;
    boost::shared_ptr<JointControlActualFsmInterface>  actualFsm;
    boost::shared_ptr<JointControlCommandFsmInterface> commandFsm;
    ros::Time                                          controlModeBadStart;
    ros::Time                                          commandModeBadStart;
    ros::Time                                          calibrationModeBadStart;
    ros::Time                                          clearFaultModeBadStart;
    std::string                                        type;
    double                                             timeLimit;

    JointControlManagerInterface(const std::string& mechanism, IoFunctions io, double timeLimit, const std::string& type)
        : JointControlCommonInterface(mechanism, io), type(type), timeLimit(timeLimit)
    {};
    virtual ~JointControlManagerInterface() {};
};

typedef boost::shared_ptr<JointControlManagerInterface> JointControlManagerPtr;
typedef std::map<std::string, JointControlManagerPtr>   JointControlManagerMap;

#endif
