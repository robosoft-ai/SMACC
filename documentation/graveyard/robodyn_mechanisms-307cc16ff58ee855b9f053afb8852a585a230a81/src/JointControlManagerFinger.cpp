/**
 * @file JointControlManagerFinger.cpp
 */

#include "nasa_robodyn_mechanisms_core/JointControlManagerFinger.h"

JointControlManagerFinger::JointControlManagerFinger(const std::string& mechanism, IoFunctions ioFunctions, double timeLimit, const std::string& type)
    : JointControlManagerInterface(mechanism, ioFunctions, timeLimit, type)
{
    // Initialize variables
    controlModeBadStart     = ros::Time::now();
    commandModeBadStart     = ros::Time::now();
    calibrationModeBadStart = ros::Time::now();

    // Instantiate state machines
    actualFsm  = boost::make_shared<JointControlActualFsmFinger> (mechanism, ioFunctions);
    commandFsm = boost::make_shared<JointControlCommandFsmFinger>(mechanism, ioFunctions);
}

JointControlManagerFinger::~JointControlManagerFinger()
{
    // Nothing
}

nasa_r2_common_msgs::JointControlData JointControlManagerFinger::getActualStates(void)
{
    nasa_r2_common_msgs::JointControlData actualStatesLocal  = actualFsm->getStates();
    nasa_r2_common_msgs::JointControlData commandStatesLocal = commandFsm->getStates();
    // Handle faking states
    if(actualStatesLocal.controlMode.state  == nasa_r2_common_msgs::JointControlMode::PARK)
    {
        switch(commandStatesLocal.controlMode.state)
        {
            case nasa_r2_common_msgs::JointControlMode::BOOTLOADER:
                actualStatesLocal.controlMode.state = commandStatesLocal.controlMode.state;
                //std::cout<<"commandstate is bootloader, do nothing?"<<std::endl;
                break;
            case nasa_r2_common_msgs::JointControlMode::OFF:
                //std::cout<<"actual state changed to off"<<std::endl;
                actualStatesLocal.controlMode.state = commandStatesLocal.controlMode.state;
                break;
            case nasa_r2_common_msgs::JointControlMode::PARK:
                //std::cout<<"actual state changed to park"<<std::endl;
                //actualStatesLocal.controlMode.state = commandStatesLocal.controlMode.state;
                break;
            case nasa_r2_common_msgs::JointControlMode::NEUTRAL:
                actualStatesLocal.controlMode.state = commandStatesLocal.controlMode.state;
                //std::cout<<"actual state changed to neutral"<<std::endl;
                break;
            case nasa_r2_common_msgs::JointControlMode::DRIVE:
                // check cal mode
                if(actualStatesLocal.calibrationMode.state == nasa_r2_common_msgs::JointControlCalibrationMode::ENABLE)
                {
                    commandFsm->park();
                    actualStatesLocal.controlMode.state = nasa_r2_common_msgs::JointControlMode::PARK;
                }
                else
                {
                    actualStatesLocal.controlMode.state = commandStatesLocal.controlMode.state;
                }
                //std::cout<<"actual state changed to drive"<<std::endl;
                break;
            default:
                // nothing
                break;
        }
    }
    
    actualStatesLocal.commandMode    = commandStatesLocal.commandMode;
    actualStatesLocal.clearFaultMode = commandStatesLocal.clearFaultMode;
    return actualStatesLocal;
}

nasa_r2_common_msgs::JointControlData JointControlManagerFinger::getCommandStates(void)
{
    return commandFsm->getStates();
}

void JointControlManagerFinger::setCommandStates(nasa_r2_common_msgs::JointControlData command)
{
    switch(command.controlMode.state)
    {
        case nasa_r2_common_msgs::JointControlMode::BOOTLOADER:
            if (actualFsm->getStates().controlMode.state != nasa_r2_common_msgs::JointControlMode::BOOTLOADER)
            {
                RCS::Logger::getCategory("gov.nasa.JointControlManagerWrist") << log4cpp::Priority::WARN << "BOOTLOADER command not allowed for " << mechanism << "; commanding park()";
                commandFsm->park();
            }
            break;
        case nasa_r2_common_msgs::JointControlMode::OFF:
            commandFsm->off();
            break;
        case nasa_r2_common_msgs::JointControlMode::PARK:
            commandFsm->park();
            break;
        case nasa_r2_common_msgs::JointControlMode::NEUTRAL:
            commandFsm->neutral();
            break;
        case nasa_r2_common_msgs::JointControlMode::DRIVE:
            commandFsm->drive();
            break;
        default:
            // nothing
            break;
    }

    switch(command.commandMode.state)
    {
        case nasa_r2_common_msgs::JointControlCommandMode::MOTCOM:
            commandFsm->motCom();
            break;
        case nasa_r2_common_msgs::JointControlCommandMode::STALLMODE:
            commandFsm->stallMode();
            break;
        case nasa_r2_common_msgs::JointControlCommandMode::MULTILOOPSTEP:
            commandFsm->multiLoopStep();
            break;
        case nasa_r2_common_msgs::JointControlCommandMode::MULTILOOPSMOOTH:
            commandFsm->multiLoopSmooth();
            break;
        case nasa_r2_common_msgs::JointControlCommandMode::ACTUATOR:
            commandFsm->actuator();
            break;
        default:
            // nothing
            break;
    }

    switch(command.calibrationMode.state)
    {
        case nasa_r2_common_msgs::JointControlCalibrationMode::DISABLE:
            commandFsm->disableCalibrationMode();
            break;
        case nasa_r2_common_msgs::JointControlCalibrationMode::ENABLE:
            commandFsm->enableCalibrationMode();
            break;
        default:
            // nothing
            break;
    }

    switch(command.clearFaultMode.state)
    {
        case nasa_r2_common_msgs::JointControlClearFaultMode::DISABLE:
            commandFsm->disableClearFaultMode();
            break;
        case nasa_r2_common_msgs::JointControlClearFaultMode::ENABLE:
            commandFsm->enableClearFaultMode();
            break;
        default:
            // nothing
            break;
    }
}

bool JointControlManagerFinger::verifyStates(void)
{
    prevActualStates = actualStates;
    actualStates     = getActualStates();
    commandStates    = getCommandStates();

    return verifyControlModeState() && verifyCommandModeState() && verifyCalibrationModeState() && verifyClearFaultModeState();
}

bool JointControlManagerFinger::verifyControlModeState(void)
{
    if(actualStates.controlMode.state == commandStates.controlMode.state)
    {
        controlModeBadStart = ros::Time::now();
        return true;
    }

    // Handle faults
    if(actualStates.controlMode.state == nasa_r2_common_msgs::JointControlMode::FAULTED)
    {
        controlModeBadStart = ros::Time::now();
        if(prevActualStates.controlMode.state != nasa_r2_common_msgs::JointControlMode::FAULTED)
        {
            RCS::Logger::getCategory("gov.nasa.JointControlManagerFinger") << log4cpp::Priority::INFO << "Joint faulted: " << mechanism;
        }
        return false;
    }

    // Handle other state mismatches
    ros::Duration timeSinceBadStart = ros::Time::now() - controlModeBadStart;
    if(timeSinceBadStart.toSec() > timeLimit)
    {
        RCS::Logger::getCategory("gov.nasa.JointControlManagerFinger") << log4cpp::Priority::INFO << "Timed out waiting for " << jointControlModeToString(commandStates.controlMode) << " received " << jointControlModeToString(actualStates.controlMode) << " on mechanism: " << mechanism;
        controlModeBadStart = ros::Time::now();
        return false;
    }
    return true;
}

bool JointControlManagerFinger::verifyCommandModeState(void)
{
    if(actualStates.commandMode.state == commandStates.commandMode.state)
    {
        commandModeBadStart = ros::Time::now();
        return true;
    }

    // Handle state mismatches
    ros::Duration timeSinceBadStart = ros::Time::now() - commandModeBadStart;
    if(timeSinceBadStart.toSec() > timeLimit)
    {
        RCS::Logger::getCategory("gov.nasa.JointControlManagerFinger") << log4cpp::Priority::INFO << "Timed out waiting for " << jointControlCommandModeToString(commandStates.commandMode) << " received " << jointControlCommandModeToString(actualStates.commandMode) << " on mechanism: " << mechanism;
        commandModeBadStart = ros::Time::now();
        return false;
    }
    return true;
}

bool JointControlManagerFinger::verifyCalibrationModeState(void)
{
    if(actualStates.calibrationMode.state == commandStates.calibrationMode.state)
    {
        calibrationModeBadStart = ros::Time::now();
        return true;
    }
    
    if (actualStates.calibrationMode.state == nasa_r2_common_msgs::JointControlCalibrationMode::DISABLE
        && prevActualStates.calibrationMode.state == nasa_r2_common_msgs::JointControlCalibrationMode::ENABLE
        && commandStates.calibrationMode.state == nasa_r2_common_msgs::JointControlCalibrationMode::ENABLE)
    {
        // transitioned, so make them match
        commandFsm->disableCalibrationMode();
        calibrationModeBadStart = ros::Time::now();
        return true;
    }                


    // Handle state mismatches
    ros::Duration timeSinceBadStart = ros::Time::now() - calibrationModeBadStart;
    if(timeSinceBadStart.toSec() > timeLimit)
    {
        RCS::Logger::getCategory("gov.nasa.JointControlManagerFinger") << log4cpp::Priority::INFO << "Timed out waiting for " << jointControlCalibrationModeToString(commandStates.calibrationMode) << " received " << jointControlCalibrationModeToString(actualStates.calibrationMode) << " on mechanism: " << mechanism;
        calibrationModeBadStart = ros::Time::now();
        return false;
    }
    return true;
}

bool JointControlManagerFinger::verifyClearFaultModeState(void)
{
    if(actualStates.clearFaultMode.state == commandStates.clearFaultMode.state)
    {
        clearFaultModeBadStart = ros::Time::now();
        return true;
    }

    // Handle state mismatches
    ros::Duration timeSinceBadStart = ros::Time::now() - clearFaultModeBadStart;
    if(timeSinceBadStart.toSec() > timeLimit)
    {
        RCS::Logger::getCategory("gov.nasa.JointControlManagerFinger") << log4cpp::Priority::INFO << "Timed out waiting for " << jointControlClearFaultModeToString(commandStates.clearFaultMode) << " received " << jointControlClearFaultModeToString(actualStates.clearFaultMode) << " on mechanism: " << mechanism;
        clearFaultModeBadStart = ros::Time::now();
        return false;
    }
    return true;
}

/***************************************************************************//**
 *
 * @brief Build the string summarizing the current faults in this mechanism
 *
 * @return std::string A string summarizing the current faults in this mechanism
 *
 ******************************************************************************/
std::string JointControlManagerFinger::buildFaultString()
{
    verifyStates();

    std::string outStr("");
    if(actualStates.controlMode.state == nasa_r2_common_msgs::JointControlMode::FAULTED)
    {
        outStr.append("FAULT");
    }
    else
    {
        outStr.append("none");
    }

    return outStr;
}
