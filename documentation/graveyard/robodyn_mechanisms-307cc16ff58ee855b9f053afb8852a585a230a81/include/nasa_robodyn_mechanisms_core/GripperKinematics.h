#ifndef GRIPPERKINEMATICS_H
#define GRIPPERKINEMATICS_H

#include <stdexcept>
#include <iostream>
#include "nasa_common_utilities/Logger.h"
#include <boost/math/constants/constants.hpp>
#include <boost/math/special_functions/fpclassify.hpp>

/**
 * @brief Useful for calculating gripper-specific kinematic values.  Based on Aaron Hulse's equations.
 */

class GripperKinematics
{
public:
    GripperKinematics() {};
    GripperKinematics(const double l1, const double l2, const double l3, const double l4, const double minSliderPosition, const double ballScrewEfficiency,
                      const double ballScrewMotionRatio, const double jawAngleZeroAdj1, const double jawAngleZeroAdj2, const double gripperLength,
                      const double motorTorqueConstant, const double overCenterAngle, const double overCenterStatusBuffer, double coulombFrictionCurrentOffset,
                      const double closeToZero, const bool beSilent = false);
    virtual ~GripperKinematics() {};

    void setParameters(const double l1, const double l2, const double l3, const double l4, const double minSliderPosition, const double ballScrewEfficiency,
                       const double ballScrewMotionRatio, const double jawAngleZeroAdj1, const double jawAngleZeroAdj2, const double gripperLength,
                       const double motorTorqueConstant, const double overCenterAngle, const double overCenterStatusBuffer, double coulombFrictionCurrentOffset,
                       const double closeToZero, const bool beSilent = false);

    double getEncoderAngle(const double& jawAngle);
    double getAngleOffSingular(const double& encoderAngle, const double& jawAngle);
    bool isOverCenter(const double& encoderAngle, const double& jawAngle);
    bool isOverCenterStatus(const double& encoderAngle, const double& jawAngle);
    double getMotorCurrentLimit(const double& encoderAngle, const double& jawAngle, const double& forceLimit);

private:
    double L1;
    double L2;
    double L3;
    double L4;
    double MIN_SLIDER_POSITION;
    double BALL_SCREW_EFFICIENCY;
    double BALL_SCREW_MOTION_RATIO;
    double BALL_SCREW_FORCE_RATIO;
    double JAW_ANGLE_ZERO_ADJ;
    double GRIPPER_LENGTH; 
    double MOTOR_TORQUE_CONSTANT;
    double OVER_CENTER_ANGLE;
    double OVER_CENTER_STATUS_BUFFER;
    double COULOMB_FRICTION_CURRENT_OFFSET;
    double CLOSE_TO_ZERO;
    bool   BE_SILENT;
};

typedef boost::shared_ptr<GripperKinematics> GripperKinematicsPtr;

#endif
