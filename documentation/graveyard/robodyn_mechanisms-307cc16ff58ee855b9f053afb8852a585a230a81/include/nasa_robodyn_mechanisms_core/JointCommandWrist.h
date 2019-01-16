/*!
 * @file  JointCommandWrist.h
 * @brief Defines the JointCommandWrist  class.
 */

#ifndef JOINTCOMMANDWRIST_H
#define JOINTCOMMANDWRIST_H

#include "nasa_robodyn_mechanisms_core/JointCommandInterface.h"
#include "nasa_r2_config_core/StringUtilities.h"
#include "nasa_robodyn_mechanisms_core/WristMechanism.h"
#include "nasa_robodyn_mechanisms_core/HallsToAngle.h"
#include "nasa_robodyn_utilities/MultiLoopController.h"
#include <memory>
#include <boost/ptr_container/ptr_array.hpp>
#include "EmbeddedSmoother.h"
#define TEST_BED

/***************************************************************************//**
 *
 * @brief This class provides the joint command interface for the wrist
 *
 ******************************************************************************/

class JointCommandWrist : public JointCommandInterface
{
public:
    JointCommandWrist(const std::string& mechanism, IoFunctions ioFunctions);
    virtual ~JointCommandWrist();

    sensor_msgs::JointState           getSimpleMeasuredState();
    sensor_msgs::JointState           getCompleteMeasuredState();
    nasa_r2_common_msgs::JointCommand getCommandedState();
    void setCommand(nasa_r2_common_msgs::JointCommand msg, nasa_r2_common_msgs::JointControlData control);
    void updateMeasuredState(nasa_r2_common_msgs::JointControlData msg);
    void setFaultState();

    nasa_r2_common_msgs::JointCapability getCapability();
    void loadCoeffs();

protected:
    std::auto_ptr<WristMechanism> wrist;
    std::auto_ptr<WristMechanism> wristFault;
    std::auto_ptr<MultiLoopController> q1Controller;
    std::auto_ptr<MultiLoopController> q2Controller;

    void limitSliderDiff(float& slider1, float& slider2);

    std::string angleRosJointName;
    std::string motorRosJointName;
    std::string encoderRosJointName;
    std::string sliderRosJointName;
    std::string hallsRosJointName;

    std::string encoderLittlePositionStatusElement;
    std::string encoderThumbPositionStatusElement;
    std::string hallsPitchStatusElement;
    std::string hallsYawStatusElement;

    float desiredQ1;
    float desiredQ2;
    float desiredPitch;
    float desiredYaw;
    float desiredPitchVel;
    float desiredYawVel;

    Eigen::Vector2f desiredSlider;
    Eigen::Vector2f desiredAngle;

    std::string desiredMotComLittleCommandElement;
    std::string desiredMotComThumbCommandElement;

    float busVoltage;

    Eigen::Vector2f slider;
    Eigen::Vector2f sliderVel;
    Eigen::Vector2f ang;
    Eigen::Vector2f angVel;
    Eigen::Vector2f encoder;
    Eigen::Vector2f encoderVel;
    Eigen::Vector2f halls;
    Eigen::Vector2f hallsVel;

    double sliderMaxPosition;
    double sliderMinPosition;
    double maxSliderDiff;

    double sliderMaxPositionFault;
    double sliderMinPositionFault;
    double maxSliderDiffFault;
    double maxSensorErrorFault;

    // filter values
    Eigen::Vector2f filteredSlider;
    Eigen::Vector2f filteredHalls;
    std::vector<double> prevPositionVals;
    double timestep;
    double positionAlpha;
    bool sliderFilterInitialized;
    bool hallFilterInitialized;
    bool prevPosInitialized;

    Eigen::Vector2f manualCalSliderPos;
    Eigen::Vector2f manualCalAng;

    //std::string nameCalibrationState;
    std::string namePitchLimit;
    std::string nameYawLimit;
    std::string nameLittlesideSliderLimit;
    std::string nameThumbsideSliderLimit;
    std::string nameSensorError;
    std::string nameSliderDiffError;
    std::string nameUseHalls;
    std::string nameCalibrationMode;
    std::string nameCoeffState;

    std::vector<HallsToAngle> hta;

    bool fwdKinFault;

    // util
    std::vector<std::string>::iterator strVecIt;
    bool calFailureReported;
    bool calCalled;
    nasa_r2_common_msgs::JointControlCalibrationMode calibrationState;

    std::vector<double> positionVals;
    std::vector<double> velocityVals;

    std::auto_ptr<EmbeddedSmoother> smootherPitch;
    std::auto_ptr<EmbeddedSmoother> smootherYaw;
    float smoothedPitch;
    float smoothedYaw;
    Eigen::Vector2d sliderVelCommand;

    unsigned int completeMessageSize;
};

#endif // JOINTCOMMANDWRIST_H
