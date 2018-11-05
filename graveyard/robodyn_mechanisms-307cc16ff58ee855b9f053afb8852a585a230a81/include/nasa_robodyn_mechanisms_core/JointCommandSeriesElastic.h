/*!
 * @file  JointCommandSeriesElastic.h
 * @brief Defines the JointCommandSeriesElastic  class.
 */

#ifndef JOINTCOMMANDSERIESELASTIC_H_
#define JOINTCOMMANDSERIESELASTIC_H_

#include "nasa_robodyn_mechanisms_core/JointCommandInterface.h"
#include "nasa_r2_config_core/StringUtilities.h"
#include "nasa_robodyn_utilities/EncoderStateCalculator.h"
#include <boost/make_shared.hpp>

class JointCommandSeriesElastic : public JointCommandInterface
{
public:
    JointCommandSeriesElastic(const std::string& mechanism, IoFunctions ioFunctions);
    virtual ~JointCommandSeriesElastic();

    sensor_msgs::JointState           getSimpleMeasuredState();
    sensor_msgs::JointState           getCompleteMeasuredState();
    nasa_r2_common_msgs::JointCommand getCommandedState();
    void setCommand(nasa_r2_common_msgs::JointCommand msg, nasa_r2_common_msgs::JointControlData);
    void updateMeasuredState(nasa_r2_common_msgs::JointControlData msg);
    void setFaultState();

    nasa_r2_common_msgs::JointCapability getCapability();
    void loadCoeffs();

protected:
    
    //! joint status
    std::string jointPositionStatusElement;
    std::string jointVelocityStatusElement;
    std::string jointCalculatedEffortElement;

    //! motor status
    std::string motorPositionStatusElement;
    std::string motorVelocityStatusElement;
    std::string motorCurrentElement;

    //! encoder status
    std::string encoderPositionStatusElement;
    std::string encoderVelocityStatusElement;

    //! joint calculated
    std::string jointPositionFilteredStatusElement;
    std::string motorPositionFilteredStatusElement;

    //! encoder calculated
    std::string encoderRawPositionStatusElement;
    std::string encoderTimeElement;
    std::string encoderTimeoutElement;

    //! halls calculated
    std::string hallPositionStatusElement;
    std::string hallRawPositionStatusElement;
    std::string hallTimeElement;
    std::string hallTimeoutElement;

    //! embedded command
    std::string embeddedCommandPositionElement;
    std::string embeddedCommandVelocityElement;
    std::string embeddedCommandEffortElement;

    std::string desiredPositionCommandElement;
    std::string desiredPositionVelocityLimitCommandElement;
    std::string feedForwardTorqueCommandElement;
    std::string proportionalGainCommandElement;
    std::string derivativeGainCommandElement;
    std::string integralGainCommandElement;
    std::string positionLoopTorqueLimitCommandElement;
    std::string positionLoopWindupLimitCommandElement;
    std::string torqueLoopVelocityLimitCommandElement;
    std::string brakePwmCommandElement;
    std::string motComLimitCommandElement;

    double        springConstant;
    double        gearStiffness;
    double        combinedStiffness;
    double        jointKinematicOffset;
    double        jointKinematicDirection;
    double        jointGearRatio;
    double        backEmfConstant;
    int32_t       incEncNow;
    int32_t       incEncPrev;
    int32_t       incEncRef;
    double        encPosRef;
    int32_t       hallEncNow;
    int32_t       hallEncPrev;
    int32_t       hallEncRef;
    double        hallPosRef;
    ros::Time     timeNow;
    ros::Time     timePrev;
    ros::Duration deltaTime;
    double        deltaTimeSec;
    double        deltaEncPos;
    double        commandRate;

    std::string motorName;
    std::string encoderName;
    std::string jointCalculatedName;
    std::string encoderCalculatedName;
    std::string hallsCalculatedName;
    std::string embeddedCommandName;

    std::vector<double> positionVals;
    std::vector<double> velocityVals;
    std::vector<double> effortVals;

    bool isInitialized;
    unsigned int completeMessageSize;

    MotorEncoderStateCalculatorPtr motorEncoderStateCalculator;
    HallsEncoderStateCalculatorPtr hallsEncoderStateCalculator;
};

#endif /* JOINTCOMMANDSERIESELASTIC_H_ */
