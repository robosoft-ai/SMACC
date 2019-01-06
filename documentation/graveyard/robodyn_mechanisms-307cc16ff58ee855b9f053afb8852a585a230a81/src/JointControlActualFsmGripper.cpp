/**
 * @file JointControlActualFsmGripper.cpp
 */

#include "nasa_robodyn_mechanisms_core/JointControlActualFsmGripper.h"
/***************************************************************************//**
 *
 * @brief Constructor for JointControlActualFsmGripper
 *
 ******************************************************************************/
JointControlActualFsmGripper::JointControlActualFsmGripper(const std::string& mechanism, IoFunctions ioFunctions, NodeRegisterManagerPtr nodeRegisterManager)
    : JointControlActualFsmInterface(mechanism, ioFunctions), nodeRegisterManager(nodeRegisterManager)
{
    setParameters();
}

JointControlActualFsmGripper::~JointControlActualFsmGripper()
{
    // Nothing
}

void JointControlActualFsmGripper::setParameters()
{
    std::string parameterFile = io.getControlFile(mechanism);

    //! Parse parameter file
    TiXmlDocument file(parameterFile.c_str());
    bool loadOkay = file.LoadFile();
    if (!loadOkay)
    {
        std::stringstream err;
        err << "Failed to load file [" << parameterFile << "]";
        RCS::Logger::log("gov.nasa.JointControlActualFsmGripper", log4cpp::Priority::FATAL, err.str());
        throw std::runtime_error(err.str());
    }
    TiXmlHandle doc(&file);
    RCS::Logger::log("gov.nasa.JointControlActualFsmGripper", log4cpp::Priority::INFO, "File [" + parameterFile + "] successfully loaded.");

    // Print the XML file's contents
    //cout << "Successfully loaded file: " << parameterFile << endl;
    //TiXmlPrinter printer;
    //printer.SetIndent("\t");
    //file.Accept(&printer);
    //cout << printer.Str() << endl;

    // Check for ApiMap
    TiXmlHandle parametersElement(doc.FirstChildElement("ApiMap"));

    if (parametersElement.ToElement())
    {
        // Status bit names
        ProcAliveStatusName       = ApiMap::getXmlElementValue(parametersElement, "ProcAliveStatus");
        CommAliveStatusName       = ApiMap::getXmlElementValue(parametersElement, "CommAliveStatus");
        JointFaultStatusName      = ApiMap::getXmlElementValue(parametersElement, "JointFaultStatus");
        CoeffsLoadedStatusName    = ApiMap::getXmlElementValue(parametersElement, "CoeffsLoadedStatus");
        MotorEnableStatusName     = ApiMap::getXmlElementValue(parametersElement, "MotorEnableStatus");
        BrakeReleaseStatusName    = ApiMap::getXmlElementValue(parametersElement, "BrakeReleaseStatus");
        BridgeEnableStatusName    = ApiMap::getXmlElementValue(parametersElement, "BridgeEnableStatus");
        MotComSourceStatusName    = ApiMap::getXmlElementValue(parametersElement, "MotComSourceStatus");
        ControlModeStatusName     = ApiMap::getXmlElementValue(parametersElement, "ControlModeStatus");
        CalibrationModeStatusName = ApiMap::getXmlElementValue(parametersElement, "CalibrationModeStatus");
        ClearFaultStatusName      = ApiMap::getXmlElementValue(parametersElement, "ClearFaultStatus");

        // Control bit names
        BootEnableControlName     = ApiMap::getXmlElementValue(parametersElement, "BootEnableControl");
    }
    else
    {
        std::stringstream err;
        err << "The file " << parameterFile << " has no element named [ApiMap]";
        RCS::Logger::log("gov.nasa.JointControlActualFsmGripper", log4cpp::Priority::ERROR, err.str());
        throw std::runtime_error(err.str());
    }
}

/***************************************************************************//**
 *
 * @brief get the Command, Control, and Calibration Mode states, as well as the Coeff State
 * @return The state information
 *
 ******************************************************************************/
nasa_r2_common_msgs::JointControlData JointControlActualFsmGripper::getStates(void)
{
    updateCommandModeState(); // Must come before updateControlModeState()
    updateControlModeState();
    updateCalibrationModeState();
    updateClearFaultModeState();
    updateCoeffState();
    return states;
}
/***************************************************************************//**
 *
 * @brief Update the control mode state
 *
 ******************************************************************************/
void JointControlActualFsmGripper::updateControlModeState(void)
{
    // Check for FPGA FAULTED bits
    if(nodeRegisterManager->getStatusValue(mechanism, ProcAliveStatusName)   == 0)
    {
        if(states.controlMode.state != nasa_r2_common_msgs::JointControlMode::FAULTED)
        {
            RCS::Logger::getCategory("gov.nasa.JointControlActualFsmGripper") << log4cpp::Priority::WARN << mechanism << ": FAULT DETECTED: " << ProcAliveStatusName << " == 0";
            RCS::Logger::getCategory("gov.nasa.JointControlActualFsmGripper") << log4cpp::Priority::DEBUG << "Transition to JointControlMode::FAULTED on joint: " << mechanism;
            states.controlMode.state = nasa_r2_common_msgs::JointControlMode::FAULTED;
        }
        return;
    }
    if(nodeRegisterManager->getStatusValue(mechanism, CommAliveStatusName)   == 0)
    {
        if(states.controlMode.state != nasa_r2_common_msgs::JointControlMode::FAULTED)
        {
            RCS::Logger::getCategory("gov.nasa.JointControlActualFsmGripper") << log4cpp::Priority::WARN << mechanism << ": FAULT DETECTED: " << CommAliveStatusName << " == 0";
            RCS::Logger::getCategory("gov.nasa.JointControlActualFsmGripper") << log4cpp::Priority::DEBUG << "Transition to JointControlMode::FAULTED on joint: " << mechanism;
            states.controlMode.state = nasa_r2_common_msgs::JointControlMode::FAULTED;
        }
        return;
    }

    // Check for BOOTLOADER state
    if(nodeRegisterManager->getControlValue(mechanism, BootEnableControlName) == 0)
    {
        if(states.controlMode.state != nasa_r2_common_msgs::JointControlMode::BOOTLOADER)
        {
            RCS::Logger::getCategory("gov.nasa.JointControlActualFsmGripper") << log4cpp::Priority::DEBUG << "Transition to JointControlMode::BOOTLOADER on joint: " << mechanism;
            states.controlMode.state = nasa_r2_common_msgs::JointControlMode::BOOTLOADER;
        }
        return;
    }

    // Check for OFF
    uint16_t currentMotorEnable  = nodeRegisterManager->getStatusValue(mechanism, MotorEnableStatusName);
    uint16_t currentBrakeRelease = nodeRegisterManager->getStatusValue(mechanism, BrakeReleaseStatusName);
    if(nodeRegisterManager->getStatusValue(mechanism, BridgeEnableStatusName) == 0)
    {
        if( (currentMotorEnable  == 0) &&
            (currentBrakeRelease == 0) )
        {
            if(states.controlMode.state != nasa_r2_common_msgs::JointControlMode::OFF)
            {
                RCS::Logger::getCategory("gov.nasa.JointControlActualFsmGripper") << log4cpp::Priority::DEBUG << "Transition to JointControlMode::OFF on joint: " << mechanism;
                states.controlMode.state = nasa_r2_common_msgs::JointControlMode::OFF;
            }
            return;
        }
        else
        {
            if(states.controlMode.state != nasa_r2_common_msgs::JointControlMode::INVALID)
            {
                RCS::Logger::getCategory("gov.nasa.JointControlActualFsmGripper") << log4cpp::Priority::ERROR << mechanism << ": INVALID CONTROL MODE STATE DETECTED, BridgeEnable == 0, and MotorEnable or BrakeRelease == 1";
                RCS::Logger::getCategory("gov.nasa.JointControlActualFsmGripper") << log4cpp::Priority::DEBUG << "Transition to JointControlMode::INVALID on joint: " << mechanism;
                states.controlMode.state = nasa_r2_common_msgs::JointControlMode::INVALID;
            }
            return;
        }
    }

    // Check for processor FAULTED bits
    if(nodeRegisterManager->getStatusValue(mechanism, JointFaultStatusName) == 1)
    {
        if(states.controlMode.state != nasa_r2_common_msgs::JointControlMode::FAULTED)
        {
            RCS::Logger::getCategory("gov.nasa.JointControlActualFsmGripper") << log4cpp::Priority::WARN << mechanism << ": FAULT DETECTED";
            RCS::Logger::getCategory("gov.nasa.JointControlActualFsmGripper") << log4cpp::Priority::DEBUG << "Transition to JointControlMode::FAULTED on joint: " << mechanism;
            states.controlMode.state = nasa_r2_common_msgs::JointControlMode::FAULTED;
        }
        return;
    }

    // Check for PARK
    if( (currentMotorEnable  == 1) &&
        (currentBrakeRelease == 0) )
    {
        if(states.controlMode.state != nasa_r2_common_msgs::JointControlMode::PARK)
        {
            RCS::Logger::getCategory("gov.nasa.JointControlActualFsmGripper") << log4cpp::Priority::DEBUG << "Transition to JointControlMode::PARK on joint: " << mechanism;
            states.controlMode.state = nasa_r2_common_msgs::JointControlMode::PARK;
        }
        return;
    }

    // Check for NEUTRAL
    if( (currentMotorEnable  == 0) &&
        (currentBrakeRelease == 1) )
    {
        if(states.controlMode.state != nasa_r2_common_msgs::JointControlMode::NEUTRAL)
        {
            RCS::Logger::getCategory("gov.nasa.JointControlActualFsmGripper") << log4cpp::Priority::DEBUG << "Transition to JointControlMode::NEUTRAL on joint: " << mechanism;
            states.controlMode.state = nasa_r2_common_msgs::JointControlMode::NEUTRAL;
        }
        return;
    }

    // Check for DRIVE
    if( (currentMotorEnable  == 1) &&
        (currentBrakeRelease == 1) )
    {
        if(states.controlMode.state != nasa_r2_common_msgs::JointControlMode::DRIVE)
        {
            RCS::Logger::getCategory("gov.nasa.JointControlActualFsmGripper") << log4cpp::Priority::DEBUG << "Transition to JointControlMode::DRIVE on joint: " << mechanism;
            states.controlMode.state = nasa_r2_common_msgs::JointControlMode::DRIVE;
        }
        return;
    }

    // All valid states have been checked, what's left is invalid
    if(states.controlMode.state != nasa_r2_common_msgs::JointControlMode::INVALID)
    {
        RCS::Logger::getCategory("gov.nasa.JointControlActualFsmGripper") << log4cpp::Priority::INFO << mechanism << ": INVALID CONTROL MODE STATE DETECTED";
        RCS::Logger::getCategory("gov.nasa.JointControlActualFsmGripper") << log4cpp::Priority::DEBUG << "Transition to JointControlMode::INVALID on joint: " << mechanism;
        states.controlMode.state = nasa_r2_common_msgs::JointControlMode::INVALID;
    }
}
/***************************************************************************//**
 *
 * @brief Update the command mode state 
 *
 ******************************************************************************/
void JointControlActualFsmGripper::updateCommandModeState(void)
{
    // Check for MOTCOM
    if(nodeRegisterManager->getStatusValue(mechanism, MotComSourceStatusName) == 0)
    {
        if(states.commandMode.state != nasa_r2_common_msgs::JointControlCommandMode::MOTCOM)
        {
            RCS::Logger::getCategory("gov.nasa.JointControlActualFsmGripper") << log4cpp::Priority::DEBUG << "Transition to JointControlCommandMode::MOTCOM on joint: " << mechanism;
            states.commandMode.state = nasa_r2_common_msgs::JointControlCommandMode::MOTCOM;
        }
        return;
    }

    uint16_t currentCoeffsLoaded = nodeRegisterManager->getStatusValue(mechanism, CoeffsLoadedStatusName);
    uint16_t currentControlMode  = nodeRegisterManager->getStatusValue(mechanism, ControlModeStatusName);

    // Check for transition from MULTI to MOTCOM
    if( (currentControlMode == 0)                                                                &&
        (states.commandMode.state == nasa_r2_common_msgs::JointControlCommandMode::MULTILOOPSTEP))
    {
        return;
    }

    // Check for STALLMODE
    if( (currentCoeffsLoaded   == 1) &&
        (currentControlMode    == 1) )
    {
        if(states.commandMode.state != nasa_r2_common_msgs::JointControlCommandMode::STALLMODE)
        {
            RCS::Logger::getCategory("gov.nasa.JointControlActualFsmGripper") << log4cpp::Priority::DEBUG << "Transition to JointControlCommandMode::STALLMODE on joint: " << mechanism;
            states.commandMode.state = nasa_r2_common_msgs::JointControlCommandMode::STALLMODE;
        }
        return;
    }

    // Check for MULTILOOPSTEP
    if( (currentCoeffsLoaded   == 1) &&
        (currentControlMode    == 3) )
    {
        if(states.commandMode.state != nasa_r2_common_msgs::JointControlCommandMode::MULTILOOPSTEP)
        {
            RCS::Logger::getCategory("gov.nasa.JointControlActualFsmGripper") << log4cpp::Priority::DEBUG << "Transition to JointControlCommandMode::MULTILOOPSTEP on joint: " << mechanism;
            states.commandMode.state = nasa_r2_common_msgs::JointControlCommandMode::MULTILOOPSTEP;
        }
        return;
    }

    // All valid states have been checked, what's left is invalid
    if(states.commandMode.state != nasa_r2_common_msgs::JointControlCommandMode::INVALID)
    {
        RCS::Logger::getCategory("gov.nasa.JointControlActualFsmGripper") << log4cpp::Priority::INFO << mechanism << ": INVALID COMMAND MODE STATE DETECTED";
        RCS::Logger::getCategory("gov.nasa.JointControlActualFsmGripper") << log4cpp::Priority::DEBUG << "Transition to JointControlCommandMode::INVALID on joint: " << mechanism;
        states.commandMode.state = nasa_r2_common_msgs::JointControlCommandMode::INVALID;
    }
}
/***************************************************************************//**
 *
 * @brief Update the calibration mode state 
 *
 ******************************************************************************/
void JointControlActualFsmGripper::updateCalibrationModeState(void)
{
    // Check for DISABLE
    if(nodeRegisterManager->getStatusValue(mechanism, CalibrationModeStatusName) == 0)
    {
        if(states.calibrationMode.state != nasa_r2_common_msgs::JointControlCalibrationMode::DISABLE)
        {
            RCS::Logger::getCategory("gov.nasa.JointControlActualFsmGripper") << log4cpp::Priority::DEBUG << "Transition to JointControlCalibrationMode::DISABLE on joint: " << mechanism;
            states.calibrationMode.state = nasa_r2_common_msgs::JointControlCalibrationMode::DISABLE;
        }
        return;
    }

    // If not DISABLE then ENABLE
    if(states.calibrationMode.state != nasa_r2_common_msgs::JointControlCalibrationMode::ENABLE)
    {
        RCS::Logger::getCategory("gov.nasa.JointControlActualFsmGripper") << log4cpp::Priority::DEBUG << "Transition to JointControlCalibrationMode::ENABLE on joint: " << mechanism;
        states.calibrationMode.state = nasa_r2_common_msgs::JointControlCalibrationMode::ENABLE;
    }
}
/***************************************************************************//**
 *
 * @brief Update the clearFault mode state 
 *
 ******************************************************************************/
void JointControlActualFsmGripper::updateClearFaultModeState(void)
{
    // Check for DISABLE
    if(nodeRegisterManager->getStatusValue(mechanism, ClearFaultStatusName) == 0)
    {
        if(states.clearFaultMode.state != nasa_r2_common_msgs::JointControlClearFaultMode::DISABLE)
        {
            RCS::Logger::getCategory("gov.nasa.JointControlActualFsmGripper") << log4cpp::Priority::DEBUG << "Transition to JointControlClearFaultMode::DISABLE on joint: " << mechanism;
            states.clearFaultMode.state = nasa_r2_common_msgs::JointControlClearFaultMode::DISABLE;
        }
        return;
    }

    // If not DISABLE then ENABLE
    if(states.clearFaultMode.state != nasa_r2_common_msgs::JointControlClearFaultMode::ENABLE)
    {
        RCS::Logger::getCategory("gov.nasa.JointControlActualFsmGripper") << log4cpp::Priority::DEBUG << "Transition to JointControlClearFaultMode::ENABLE on joint: " << mechanism;
        states.clearFaultMode.state = nasa_r2_common_msgs::JointControlClearFaultMode::ENABLE;
    }
}
/***************************************************************************//**
 *
 * @brief Update the coefficient state 
 *
 ******************************************************************************/
void JointControlActualFsmGripper::updateCoeffState(void)
{
    // Check for NOTLOADED
    if(nodeRegisterManager->getStatusValue(mechanism, CoeffsLoadedStatusName) == 0)
    {
        if(states.coeffState.state != nasa_r2_common_msgs::JointControlCoeffState::NOTLOADED)
        {
            RCS::Logger::getCategory("gov.nasa.JointControlActualFsmGripper") << log4cpp::Priority::DEBUG << "Transition to JointControlCoeffState::NOTLOADED on joint: " << mechanism;
            states.coeffState.state = nasa_r2_common_msgs::JointControlCoeffState::NOTLOADED;
        }
        return;
    }

    // If not NOTLOADED then LOADED
    if(states.coeffState.state != nasa_r2_common_msgs::JointControlCoeffState::LOADED)
    {
        RCS::Logger::getCategory("gov.nasa.JointControlActualFsmGripper") << log4cpp::Priority::DEBUG << "Transition to JointControlCoeffState::LOADED on joint: " << mechanism;
        states.coeffState.state = nasa_r2_common_msgs::JointControlCoeffState::LOADED;
    }
}
