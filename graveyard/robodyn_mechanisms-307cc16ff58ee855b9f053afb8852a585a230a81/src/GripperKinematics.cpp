#include "nasa_robodyn_mechanisms_core/GripperKinematics.h"

using namespace std;
using namespace log4cpp;

/**
 * @brief All distance values are provided in inches, and converted to meters in the constructor.
 */
GripperKinematics::GripperKinematics(const double l1, const double l2, const double l3, const double l4, const double minSliderPosition, const double ballScrewEfficiency,
                                     const double ballScrewMotionRatio, const double jawAngleZeroAdj1, const double jawAngleZeroAdj2, const double gripperLength,
                                     const double motorTorqueConstant, const double overCenterAngle, const double overCenterStatusBuffer, double coulombFrictionCurrentOffset,
                                     const double closeToZero, const bool beSilent)
{
    setParameters(l1, l2, l3, l4, minSliderPosition, ballScrewEfficiency,
                  ballScrewMotionRatio, jawAngleZeroAdj1, jawAngleZeroAdj2, gripperLength,
                  motorTorqueConstant, overCenterAngle, overCenterStatusBuffer, coulombFrictionCurrentOffset,
                  closeToZero, beSilent);
}

void GripperKinematics::setParameters(const double l1, const double l2, const double l3, const double l4, const double minSliderPosition, const double ballScrewEfficiency,
                                      const double ballScrewMotionRatio, const double jawAngleZeroAdj1, const double jawAngleZeroAdj2, const double gripperLength,
                                      const double motorTorqueConstant, const double overCenterAngle, const double overCenterStatusBuffer, double coulombFrictionCurrentOffset,
                                      const double closeToZero, const bool beSilent)
{
    L1                              = l1 * 0.0254;
    L2                              = l2 * 0.0254;
    L3                              = l3 * 0.0254;
    L4                              = l4 * 0.0254;
    MIN_SLIDER_POSITION             = minSliderPosition * 0.0254;
    BALL_SCREW_EFFICIENCY           = ballScrewEfficiency;
    BALL_SCREW_MOTION_RATIO         = ballScrewMotionRatio / (2.0 * boost::math::constants::pi<double>());
    BALL_SCREW_FORCE_RATIO          = BALL_SCREW_EFFICIENCY/BALL_SCREW_MOTION_RATIO;
    JAW_ANGLE_ZERO_ADJ              = std::atan2(jawAngleZeroAdj1, jawAngleZeroAdj2);
    GRIPPER_LENGTH                  = gripperLength * 0.0254;
    MOTOR_TORQUE_CONSTANT           = motorTorqueConstant;
    OVER_CENTER_ANGLE               = overCenterAngle;
    OVER_CENTER_STATUS_BUFFER       = overCenterStatusBuffer;
    COULOMB_FRICTION_CURRENT_OFFSET = coulombFrictionCurrentOffset;
    CLOSE_TO_ZERO                   = closeToZero;
    BE_SILENT                       = beSilent;
}

/**
 * @brief Calculates encoder angle using a 3rd order polynomial.
 */
double GripperKinematics::getEncoderAngle(const double& jawAngle)
{
    return -( (92.7711 * pow(-jawAngle, 3)) + (121.2354 * pow(-jawAngle, 2)) + (80.4027 * -jawAngle) - 17.229 );
}

/**
 * @brief Calculates the "over center value" gamma.
 */
double GripperKinematics::getAngleOffSingular(const double& encoderAngle, const double& jawAngle)
{
    double x                  = (-encoderAngle * BALL_SCREW_MOTION_RATIO) + MIN_SLIDER_POSITION;
    double d                  = pow((pow(x,2) + pow(L4,2)), 0.5);
    double sliderAngle        = atan2(x, L4);
    if (boost::math::isnan(sliderAngle))
    {
        if (not BE_SILENT)
        {
            BE_SILENT = true;
            stringstream err;
            err << "getAngleOffSingular() - sliderAngle is NaN - atan2(" << x << ", " << L4 << ")";
            RCS::Logger::log("gov.nasa.robodyn.GripperKinematics", Priority::ERROR, err.str());
            throw runtime_error(err.str());
        }
        else
        {
            return -1;
        }
    }
    double theta2             = boost::math::constants::pi<double>() - (JAW_ANGLE_ZERO_ADJ + sliderAngle + jawAngle);
    double a_2                = pow(d,2) + pow(L3,2) - (2 * d * L3 * cos(theta2));
    double a                  = pow(a_2,0.5);
    double alpha              = acos((pow(L3,2) - pow(d,2) - a_2) / (-2 * d * a));
    if (boost::math::isnan(alpha))
    {
        if (not BE_SILENT)
        {
            BE_SILENT = true;
            stringstream err;
            err << "getAngleOffSingular() - alpha is NaN - acos(" << (pow(L3,2) - pow(d,2) - a_2) / (-2 * d * a) << ")";
            RCS::Logger::log("gov.nasa.robodyn.GripperKinematics", Priority::ERROR, err.str());
            throw runtime_error(err.str());
        }
        else
        {
            return -1;
        }
    }
    double beta               = acos((pow(L2,2) - pow(L1,2) - a_2) / (-2 * L1 * a));
    if (boost::math::isnan(beta))
    {
        if (not BE_SILENT)
        {
            BE_SILENT = true;
            stringstream err;
            err << "getAngleOffSingular() - beta is NaN - acos(" << (pow(L2,2) - pow(L1,2) - a_2) / (-2 * L1 * a) << ")";
            RCS::Logger::log("gov.nasa.robodyn.GripperKinematics", Priority::ERROR, err.str());
            throw runtime_error(err.str());
        }
        else
        {
            return -1;
        }
    }
    double theta1             = alpha + beta;
    double gamma              = acos((pow(d,2) + pow(L3,2) - pow(L1,2) - pow(L2,2) - (2 * d * L3 * cos(theta2))) / (-2 * L1 * L2));
    if (boost::math::isnan(gamma))
    {
        if (not BE_SILENT)
        {
            BE_SILENT = true;
            stringstream err;
            err << "getAngleOffSingular() - gamma is NaN - acos(" << (pow(d,2) + pow(L3,2) - pow(L1,2) - pow(L2,2) - (2 * d * L3 * cos(theta2))) / (-2 * L1 * L2) << ")";
            RCS::Logger::log("gov.nasa.robodyn.GripperKinematics", Priority::ERROR, err.str());
            throw runtime_error(err.str());
        }
        else
        {
            return -1;
        }
    }
    double theta3             = theta1 - sliderAngle;

    BE_SILENT = false;
    return theta3;
}

/**
 * @brief Determines whether the gripper is mechanically over center.
 */
bool GripperKinematics::isOverCenter(const double& encoderAngle, const double& jawAngle)
{
    return (getAngleOffSingular(encoderAngle, jawAngle) < OVER_CENTER_ANGLE);
}

/**
 * @brief Determines whether the gripper is mechanically over center.
 */
bool GripperKinematics::isOverCenterStatus(const double& encoderAngle, const double& jawAngle)
{
    return (getAngleOffSingular(encoderAngle, jawAngle) < (OVER_CENTER_ANGLE + OVER_CENTER_STATUS_BUFFER));
}

/**
 * @brief Calculates the motor current limit given the current orientation of the gripper.
 */
double GripperKinematics::getMotorCurrentLimit(const double& encoderAngle, const double& jawAngle, const double& forceLimit)
{
    double x                  = (-encoderAngle * BALL_SCREW_MOTION_RATIO) + MIN_SLIDER_POSITION;
    double d                  = pow((pow(x,2) + pow(L4,2)), 0.5);
    double sliderAngle        = atan2(x, L4);
    if (boost::math::isnan(sliderAngle))
    {
        stringstream err;
        err << "getMotorCurrentLimit() - sliderAngle is NaN - atan2(" << x << ", " << L4 << ")";
        RCS::Logger::log("gov.nasa.robodyn.GripperKinematics", Priority::ERROR, err.str());
        throw runtime_error(err.str());
    }
    double theta2             = boost::math::constants::pi<double>() - (JAW_ANGLE_ZERO_ADJ + sliderAngle + jawAngle);
    double a_2                = pow(d,2) + pow(L3,2) - (2 * d * L3 * cos(theta2));
    double a                  = pow(a_2,0.5);
    double alpha              = acos((pow(L3,2) - pow(d,2) - a_2) / (-2 * d * a));
    if (boost::math::isnan(alpha))
    {
        stringstream err;
        err << "getMotorCurrentLimit() - alpha is NaN - acos(" << (pow(L3,2) - pow(d,2) - a_2) / (-2 * d * a) << ")";
        RCS::Logger::log("gov.nasa.robodyn.GripperKinematics", Priority::ERROR, err.str());
        throw runtime_error(err.str());
    }
    double beta               = acos((pow(L2,2) - pow(L1,2) - a_2) / (-2 * L1 * a));
    if (boost::math::isnan(beta))
    {
        stringstream err;
        err << "getMotorCurrentLimit() - beta is NaN - acos(" << (pow(L2,2) - pow(L1,2) - a_2) / (-2 * L1 * a) << ")";
        RCS::Logger::log("gov.nasa.robodyn.GripperKinematics", Priority::ERROR, err.str());
        throw runtime_error(err.str());
    }
    double theta1             = alpha + beta;
    double gamma              = acos((pow(d,2) + pow(L3,2) - pow(L1,2) - pow(L2,2) - (2 * d * L3 * cos(theta2))) / (-2 * L1 * L2));
    if (boost::math::isnan(gamma))
    {
        stringstream err;
        err << "getMotorCurrentLimit() - gamma is NaN - acos(" << (pow(d,2) + pow(L3,2) - pow(L1,2) - pow(L2,2) - (2 * d * L3 * cos(theta2))) / (-2 * L1 * L2) << ")";
        RCS::Logger::log("gov.nasa.robodyn.GripperKinematics", Priority::ERROR, err.str());
        throw runtime_error(err.str());
    }
    double mu                 = (2 * boost::math::constants::pi<double>()) - theta2 - theta1 - gamma;
    double theta3             = theta1 - sliderAngle;
    double gripperTorqueLimit = forceLimit * GRIPPER_LENGTH;
    double F2                 = gripperTorqueLimit / (L3 * sin(mu));
    double FBallscrew         = F2 * sin(gamma) / cos(theta3);
    double motorTorque        = FBallscrew / BALL_SCREW_FORCE_RATIO;
    double motorCurrent       = motorTorque / MOTOR_TORQUE_CONSTANT;
    double motorCurrentLimit  = std::abs(motorCurrent) + COULOMB_FRICTION_CURRENT_OFFSET;
    if (boost::math::isnan(motorCurrentLimit))
    {
        stringstream err;
        err << "getMotorCurrentLimit() - motorCurrentLimit is NaN";
        RCS::Logger::log("gov.nasa.robodyn.GripperKinematics", Priority::ERROR, err.str());
        throw runtime_error(err.str());
    }
    
    //std::cout << "enc: " << encoderAngle << " jaw: " << jawAngle << " fLim: " << forceLimit << " mcLim: " << motorCurrentLimit << " theta3: " << theta3 << std::endl;

    return motorCurrentLimit;
}
