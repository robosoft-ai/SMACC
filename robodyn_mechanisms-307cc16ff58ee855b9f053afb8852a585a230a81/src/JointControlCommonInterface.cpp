#include "nasa_robodyn_mechanisms_core/JointControlCommonInterface.h"

std::string JointControlCommonInterface::jointControlModeToString(nasa_r2_common_msgs::JointControlMode controlMode)
{
    std::string str;
    switch(controlMode.state)
    {
        case nasa_r2_common_msgs::JointControlMode::IGNORE:
            str = "IGNORE";
            break;
        case nasa_r2_common_msgs::JointControlMode::INVALID:
            str = "INVALID";
            break;
        case nasa_r2_common_msgs::JointControlMode::BOOTLOADER:
            str = "BOOTLOADER";
            break;
        case nasa_r2_common_msgs::JointControlMode::FAULTED:
            str = "FAULTED";
            break;
        case nasa_r2_common_msgs::JointControlMode::OFF:
            str = "OFF";
            break;
        case nasa_r2_common_msgs::JointControlMode::PARK:
            str = "PARK";
            break;
        case nasa_r2_common_msgs::JointControlMode::NEUTRAL:
            str = "NEUTRAL";
            break;
        case nasa_r2_common_msgs::JointControlMode::DRIVE:
            str = "DRIVE";
            break;
        default:
            str = "UNDEFINED";
    }
    return str;
}

std::string JointControlCommonInterface::jointControlCommandModeToString(nasa_r2_common_msgs::JointControlCommandMode commandMode)
{
    std::string str;
    switch(commandMode.state)
    {
        case nasa_r2_common_msgs::JointControlCommandMode::IGNORE:
            str = "IGNORE";
            break;
        case nasa_r2_common_msgs::JointControlCommandMode::INVALID:
            str = "INVALID";
            break;
        case nasa_r2_common_msgs::JointControlCommandMode::MOTCOM:
            str = "MOTCOM";
            break;
        case nasa_r2_common_msgs::JointControlCommandMode::MULTILOOPSTEP:
            str = "MULTILOOPSTEP";
            break;
        case nasa_r2_common_msgs::JointControlCommandMode::MULTILOOPSMOOTH:
            str = "MULTILOOPSMOOTH";
            break;
        case nasa_r2_common_msgs::JointControlCommandMode::ACTUATOR:
            str = "ACTUATOR";
            break;
        default:
            str = "UNDEFINED";
    }
    return str;
}

std::string JointControlCommonInterface::jointControlCalibrationModeToString(nasa_r2_common_msgs::JointControlCalibrationMode calibrationMode)
{
    std::string str;
    switch(calibrationMode.state)
    {
        case nasa_r2_common_msgs::JointControlCalibrationMode::IGNORE:
            str = "IGNORE";
            break;
        case nasa_r2_common_msgs::JointControlCalibrationMode::DISABLE:
            str = "DISABLE";
            break;
        case nasa_r2_common_msgs::JointControlCalibrationMode::ENABLE:
            str = "ENABLE";
            break;
        default:
            str = "UNDEFINED";
    }
    return str;
}

std::string JointControlCommonInterface::jointControlClearFaultModeToString(nasa_r2_common_msgs::JointControlClearFaultMode clearFaultMode)
{
    std::string str;
    switch(clearFaultMode.state)
    {
        case nasa_r2_common_msgs::JointControlClearFaultMode::IGNORE:
            str = "IGNORE";
            break;
        case nasa_r2_common_msgs::JointControlClearFaultMode::DISABLE:
            str = "DISABLE";
            break;
        case nasa_r2_common_msgs::JointControlClearFaultMode::ENABLE:
            str = "ENABLE";
            break;
        default:
            str = "UNDEFINED";
    }
    return str;
}
