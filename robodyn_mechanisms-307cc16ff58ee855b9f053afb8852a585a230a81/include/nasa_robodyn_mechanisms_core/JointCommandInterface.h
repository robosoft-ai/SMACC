/*!
 * @file  JointCommandInterface.h
 * @brief Defines the JointCommand* interface class, establishing the JointCommand* API.
 */

#ifndef JOINTCOMMANDINTERFACE_H_
#define JOINTCOMMANDINTERFACE_H_

#include <vector>
#include <boost/function.hpp>
#include <sensor_msgs/JointState.h>
#include <nasa_r2_common_msgs/JointCommand.h>
#include <nasa_r2_common_msgs/JointCapability.h>
#include <nasa_r2_common_msgs/JointControlDataArray.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include "nasa_common_utilities/Logger.h"
#include <cstring>
#include <boost/shared_ptr.hpp>
#include "nasa_r2_config_core/ApiMap.h"

class JointCommandInterface
{
public:
    class IoFunctions
    {
    public:
        boost::function<uint16_t(const std::string&)>                 getUInt16;
        boost::function<void(const std::string&, uint16_t)>           setUInt16;
        boost::function<uint32_t(const std::string&)>                 getUInt32;
        boost::function<void(const std::string&, uint32_t)>           setUInt32;
        boost::function<int16_t(const std::string&)>                  getInt16;
        boost::function<void(const std::string&, int16_t)>            setInt16;
        boost::function<int32_t(const std::string&)>                  getInt32;
        boost::function<void(const std::string&, int32_t)>            setInt32;
        boost::function<float(const std::string&)>                    getFloat;
        boost::function<void(const std::string&, float)>              setFloat;
        boost::function<float(const std::string&)>                    getMotorCoeff;
        boost::function<bool(const std::string&)>                     hasBrainstemCoeff;
        boost::function<float(const std::string&)>                    getBrainstemCoeff;
        boost::function<std::vector<std::string>(const std::string&)> getJointNames;
        boost::function<std::vector<std::string>(const std::string&)> getActuatorNames;
        boost::function<std::string(const std::string&)>              getCommandFile;
        boost::function<bool(const std::string&)>                     hasLiveCoeff;
        boost::function<float(const std::string&)>                    getLiveCoeff;
        boost::function<void(const std::string&, float)>              setLiveCoeff;
    };

    virtual sensor_msgs::JointState           getSimpleMeasuredState() = 0;
    virtual sensor_msgs::JointState           getCompleteMeasuredState() = 0;
    virtual nasa_r2_common_msgs::JointCommand getCommandedState() = 0;
    virtual void setCommand(nasa_r2_common_msgs::JointCommand msg, nasa_r2_common_msgs::JointControlData control) = 0;
    virtual void updateMeasuredState(nasa_r2_common_msgs::JointControlData msg) = 0;

    virtual nasa_r2_common_msgs::JointCapability getCapability() = 0;
    virtual void loadCoeffs() = 0;

    std::string mechanism;
    std::vector<std::string> roboDynJoints;
    std::vector<std::string> roboDynActuators;

protected:
    JointCommandInterface(const std::string& mechanism, IoFunctions io)
        : mechanism(mechanism), io(io)
    {};
    virtual ~JointCommandInterface() {};

    IoFunctions io;

    sensor_msgs::JointState              simpleMeasuredStateMsg;
    sensor_msgs::JointState              completeMeasuredStateMsg;
    nasa_r2_common_msgs::JointCommand    commandedStateMsg;
    nasa_r2_common_msgs::JointCapability jointCapabilityMsg;
};

typedef boost::shared_ptr<JointCommandInterface> JointCommandInterfacePtr;

#endif /* JOINTCOMMANDINTERFACE_H_ */
