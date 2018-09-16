/***************************************************************************//**
 *
 * @file JointControlCommonInterface.h
 *
 * @brief This file contains the declaration of the JointControlCommonInterface interface.
 *
 ******************************************************************************/
#ifndef JOINTCONTROLCOMMONINTERFACE_H_
#define JOINTCONTROLCOMMONINTERFACE_H_

#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include <boost/function.hpp>
#include <nasa_r2_common_msgs/JointControlData.h>
#include <string>
#include "nasa_common_utilities/Logger.h"
#include "nasa_r2_config_core/ApiMap.h"
#include "nasa_r2_config_core/StringUtilities.h"

class JointControlCommonInterface
{
public:
    class IoFunctions
    {
    public:
        boost::function<std::string(const std::string&)> getRegisterFile;
        boost::function<std::string(const std::string&)> getControlFile;
        boost::function<bool(const std::string&)>        hasLiveCoeff;
        boost::function<float(const std::string&)>       getLiveCoeff;
        boost::function<void(const std::string&, float)> setLiveCoeff;
    };

    std::string mechanism;
    static std::string jointControlModeToString(nasa_r2_common_msgs::JointControlMode);
    static std::string jointControlCommandModeToString(nasa_r2_common_msgs::JointControlCommandMode);
    static std::string jointControlCalibrationModeToString(nasa_r2_common_msgs::JointControlCalibrationMode);
    static std::string jointControlClearFaultModeToString(nasa_r2_common_msgs::JointControlClearFaultMode);
 
protected:
    JointControlCommonInterface(const std::string& mechanism, IoFunctions io)
        : mechanism(mechanism), io(io)
    {};
    virtual ~JointControlCommonInterface() {};

    IoFunctions io;
};

#endif
