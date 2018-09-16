/*!
 * @file  JointCommandRigid.h
 * @brief Defines the JointCommandRigid  class.
 */

#ifndef JOINTCOMMANDRIGID_H_
#define JOINTCOMMANDRIGID_H_

#include "nasa_robodyn_mechanisms_core/JointCommandInterface.h"
#include "nasa_r2_config_core/StringUtilities.h"

class JointCommandRigid : public JointCommandInterface
{
public:
    JointCommandRigid(const std::string& mechanism, IoFunctions ioFunctions);
    virtual ~JointCommandRigid();

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
    std::string encoderRoboDynJointName;

    std::string jointPositionStatusElement;
    std::string encoderPositionStatusElement;

    std::string jointVelocityStatusElement;
    std::string encoderVelocityStatusElement;

    std::string desiredPositionCommandElement;
    std::string desiredPositionVelocityLimitCommandElement;
    std::string brakePwmCommandElement;
    std::string motComLimitCommandElement;

    double jointKinematicOffset;

    std::string encoderName;

    std::vector<double> positionVals;
    std::vector<double> velocityVals;
    std::vector<double> effortVals;

    unsigned int completeMessageSize;
};

#endif /* JOINTCOMMANDSERIESELASTIC_H_ */
