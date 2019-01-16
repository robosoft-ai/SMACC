/*!
 * @file  JointCommandGripper.h
 * @brief Defines the JointCommandGripper  class.
 */

#ifndef JOINTCOMMANDGRIPPER_H_
#define JOINTCOMMANDGRIPPER_H_

#include "nasa_robodyn_mechanisms_core/JointCommandInterface.h"
#include "nasa_r2_config_core/StringUtilities.h"

class JointCommandGripper : public JointCommandInterface
{
public:
    JointCommandGripper(const std::string& mechanism, IoFunctions ioFunctions);
    virtual ~JointCommandGripper();

    sensor_msgs::JointState           getSimpleMeasuredState();
    sensor_msgs::JointState           getCompleteMeasuredState();
    nasa_r2_common_msgs::JointCommand getCommandedState();
    void setCommand(nasa_r2_common_msgs::JointCommand msg, nasa_r2_common_msgs::JointControlData);
    void updateMeasuredState(nasa_r2_common_msgs::JointControlData msg);
    void setFaultState();

    nasa_r2_common_msgs::JointCapability getCapability();
    void loadCoeffs();

protected:
    std::string jointRoboDynJointName;
    std::string jawLeftRoboDynJointName;
    std::string jawRightRoboDynJointName;

    std::string jointPositionStatusElement;
    std::string jawLeftPositionStatusElement;
    std::string jawRightPositionStatusElement;

    std::string jointVelocityStatusElement;
    std::string jawLeftVelocityStatusElement;
    std::string jawRightVelocityStatusElement;

    std::string jawLeftTorqueStatusElement;
    std::string jawRightTorqueStatusElement;

    std::string desiredPositionCommandElement;
    std::string desiredPositionVelocityLimitCommandElement;
    std::string feedForwardTorqueCommandElement;
    std::string proportionalGainCommandElement;
    std::string derivativeGainCommandElement;
    std::string integralGainCommandElement;
    std::string positionLoopTorqueLimitCommandElement;
    std::string positionLoopWindupLimitCommandElement;
    std::string torqueLoopVelocityLimitCommandElement;

    std::string jawLeftName;
    std::string jawRightName;

    std::vector<double> positionVals;
    std::vector<double> velocityVals;
    std::vector<double> effortVals;

    unsigned int completeMessageSize;
};

#endif /* JOINTCOMMANDGRIPPER_H_ */
