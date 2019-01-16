/**
 * @file JointControlCommandFsmGripper.cpp
 */

#include "nasa_robodyn_mechanisms_core/JointControlCommandFsmGripper.h"
/***************************************************************************//**
 *
 * @brief Constructor for JointControlCommandFsmGripper
 *
 ******************************************************************************/
JointControlCommandFsmGripper::JointControlCommandFsmGripper(const std::string& mechanism, IoFunctions ioFunctions, NodeRegisterManagerPtr nodeRegisterManager)
    : JointControlCommandFsmInterface(mechanism, ioFunctions), nodeRegisterManager(nodeRegisterManager)
{
    setParameters();
    bootLoader();
    motCom();
    disableCalibrationMode();
    disableClearFaultMode();
}

JointControlCommandFsmGripper::~JointControlCommandFsmGripper()
{
    // Nothing
}
void JointControlCommandFsmGripper::setParameters()
{
    std::string parameterFile = io.getControlFile(mechanism);

    //! Parse parameter file
    TiXmlDocument file(parameterFile.c_str());
    bool loadOkay = file.LoadFile();
    if (!loadOkay)
    {
        std::stringstream err;
        err << "Failed to load file [" << parameterFile << "]";
        RCS::Logger::log("gov.nasa.JointControlCommandFsmGripper", log4cpp::Priority::FATAL, err.str());
        throw std::runtime_error(err.str());
    }
    TiXmlHandle doc(&file);
    RCS::Logger::log("gov.nasa.JointControlCommandFsmGripper", log4cpp::Priority::INFO, "File [" + parameterFile + "] successfully loaded.");

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
        CoeffsLoadedStatusName     = ApiMap::getXmlElementValue(parametersElement, "CoeffsLoadedStatus");

        // Control bit names
        BootEnableControlName      = ApiMap::getXmlElementValue(parametersElement, "BootEnableControl");
        BridgeEnableControlName    = ApiMap::getXmlElementValue(parametersElement, "BridgeEnableControl");
        BrakeReleaseControlName    = ApiMap::getXmlElementValue(parametersElement, "BrakeReleaseControl");
        MotorEnableControlName     = ApiMap::getXmlElementValue(parametersElement, "MotorEnableControl");
        MotComSourceControlName    = ApiMap::getXmlElementValue(parametersElement, "MotComSourceControl");
        ControlModeControlName     = ApiMap::getXmlElementValue(parametersElement, "ControlModeControl");
        CalibrationModeControlName = ApiMap::getXmlElementValue(parametersElement, "CalibrationModeControl");
        ClearFaultControlName      = ApiMap::getXmlElementValue(parametersElement, "ClearFaultControl");
    }
    else
    {
        std::stringstream err;
        err << "The file " << parameterFile << " has no element named [ApiMap]";
        RCS::Logger::log("gov.nasa.JointControlCommandFsmGripper", log4cpp::Priority::ERROR, err.str());
        throw std::runtime_error(err.str());
    }
}

/***************************************************************************//**
 *
 * @brief Retrieve state information
 * @return The state information
 *
 ******************************************************************************/
nasa_r2_common_msgs::JointControlData JointControlCommandFsmGripper::getStates(void)
{
    return states;
}

// controlMode functions
/***************************************************************************//**
 *
 * @brief Transition command to JointControlMode::BOOTLOADER for the joint
 *
 ******************************************************************************/
void JointControlCommandFsmGripper::bootLoader(void)
{
    RCS::Logger::getCategory("gov.nasa.JointControlCommandFsmGripper") << log4cpp::Priority::DEBUG << "Command transition to JointControlMode::BOOTLOADER successful on joint: " << mechanism;
    nodeRegisterManager->setControlValue(mechanism, BootEnableControlName,   0);
    nodeRegisterManager->setControlValue(mechanism, BridgeEnableControlName, 0);
    nodeRegisterManager->setControlValue(mechanism, BrakeReleaseControlName, 0);
    nodeRegisterManager->setControlValue(mechanism, MotorEnableControlName,  0);
    states.controlMode.state = nasa_r2_common_msgs::JointControlMode::BOOTLOADER;
}
/***************************************************************************//**
 *
 * @brief Transition command to JointControlMode::OFF for the joint
 *
 ******************************************************************************/
void JointControlCommandFsmGripper::off(void)
{
    RCS::Logger::getCategory("gov.nasa.JointControlCommandFsmGripper") << log4cpp::Priority::DEBUG << "Command transition to JointControlMode::OFF successful on joint: " << mechanism;
    nodeRegisterManager->setControlValue(mechanism, BootEnableControlName,   1);
    nodeRegisterManager->setControlValue(mechanism, BridgeEnableControlName, 0);
    nodeRegisterManager->setControlValue(mechanism, BrakeReleaseControlName, 0);
    nodeRegisterManager->setControlValue(mechanism, MotorEnableControlName,  0);
    states.controlMode.state = nasa_r2_common_msgs::JointControlMode::OFF;
}
/***************************************************************************//**
 *
 * @brief Transition command to JointControlMode::PARK for the joint
 *
 ******************************************************************************/
void JointControlCommandFsmGripper::park(void)
{
    RCS::Logger::getCategory("gov.nasa.JointControlCommandFsmGripper") << log4cpp::Priority::DEBUG << "Command transition to JointControlMode::PARK successful on joint: " << mechanism;
    nodeRegisterManager->setControlValue(mechanism, BridgeEnableControlName, 1);
    nodeRegisterManager->setControlValue(mechanism, BrakeReleaseControlName, 0);
    nodeRegisterManager->setControlValue(mechanism, MotorEnableControlName,  1);
    states.controlMode.state = nasa_r2_common_msgs::JointControlMode::PARK;
}
/***************************************************************************//**
 *
 * @brief Transition command to JointControlMode::NEUTRAL for the joint
 *
 ******************************************************************************/
void JointControlCommandFsmGripper::neutral(void)
{
    RCS::Logger::getCategory("gov.nasa.JointControlCommandFsmGripper") << log4cpp::Priority::DEBUG << "Command transition to JointControlMode::NEUTRAL successful on joint: " << mechanism;
    nodeRegisterManager->setControlValue(mechanism, BridgeEnableControlName, 1);
    nodeRegisterManager->setControlValue(mechanism, BrakeReleaseControlName, 1);
    nodeRegisterManager->setControlValue(mechanism, MotorEnableControlName,  0);
    states.controlMode.state = nasa_r2_common_msgs::JointControlMode::NEUTRAL;
}
/***************************************************************************//**
 *
 * @brief Transition command to JointControlMode::DRIVE for the joint
 *
 ******************************************************************************/
void JointControlCommandFsmGripper::drive(void)
{
    RCS::Logger::getCategory("gov.nasa.JointControlCommandFsmGripper") << log4cpp::Priority::DEBUG << "Command transition to JointControlMode::DRIVE successful on joint: " << mechanism;
    nodeRegisterManager->setControlValue(mechanism, BrakeReleaseControlName, 1);
    nodeRegisterManager->setControlValue(mechanism, MotorEnableControlName,  1);
    states.controlMode.state = nasa_r2_common_msgs::JointControlMode::DRIVE;
}

// commandMode functions
/***************************************************************************//**
 *
 * @brief Transition command to JointControlMode::MOTCOM for the joint
 *
 ******************************************************************************/
void JointControlCommandFsmGripper::motCom(void)
{
    RCS::Logger::getCategory("gov.nasa.JointControlCommandFsmGripper") << log4cpp::Priority::DEBUG << "Command transition to JointControlMode::MOTCOM successful on joint: " << mechanism;
    nodeRegisterManager->setControlValue(mechanism, MotComSourceControlName, 0); // Brainstem mot com
    nodeRegisterManager->setControlValue(mechanism, ControlModeControlName,  0); // MotCom control mode
    states.commandMode.state = nasa_r2_common_msgs::JointControlCommandMode::MOTCOM;
}
/***************************************************************************//**
 *
 * @brief Transition command to JointControlCommandMode::STALLMODE for the joint
 *
 ******************************************************************************/
void JointControlCommandFsmGripper::stallMode(void)
{
    RCS::Logger::getCategory("gov.nasa.JointControlCommandFsmGripper") << log4cpp::Priority::DEBUG << "Command transition to JointControlCommandMode::STALLMODE successful on mechanism: " << mechanism;
    states.commandMode.state = nasa_r2_common_msgs::JointControlCommandMode::STALLMODE;

    if(nodeRegisterManager->getStatusValue(mechanism, CoeffsLoadedStatusName) == 1)
    {
        RCS::Logger::getCategory("gov.nasa.JointControlCommandFsmGripper") << log4cpp::Priority::DEBUG << "Command transition to JointControlMode::STALLMODE successful on joint: " << mechanism;

        nodeRegisterManager->setControlValue(mechanism, MotComSourceControlName, 1); // Embedded mot com
        nodeRegisterManager->setControlValue(mechanism, ControlModeControlName,  1); // StallMode control mode

        states.commandMode.state = nasa_r2_common_msgs::JointControlCommandMode::STALLMODE;
        return;
    }
    RCS::Logger::getCategory("gov.nasa.JointControlCommandFsmGripper") << log4cpp::Priority::WARN << "Cannot transition to STALLMODE, coeffs not loaded on joint: " << mechanism;
    RCS::Logger::getCategory("gov.nasa.JointControlCommandFsmGripper") << log4cpp::Priority::DEBUG << "Coeffs not loaded, transition to STALLMODE not allowed on joint: " << mechanism;
}
/***************************************************************************//**
 *
 * @brief Transition command to JointControlMode::MULTILOOPSTEP for the joint
 *
 ******************************************************************************/
void JointControlCommandFsmGripper::multiLoopStep(void)
{
    if(nodeRegisterManager->getStatusValue(mechanism, CoeffsLoadedStatusName) == 1)
    {
        RCS::Logger::getCategory("gov.nasa.JointControlCommandFsmGripper") << log4cpp::Priority::DEBUG << "Command transition to JointControlMode::MULTILOOPSTEP successful on joint: " << mechanism;

        nodeRegisterManager->setControlValue(mechanism, MotComSourceControlName,   1); // Embedded mot com
        nodeRegisterManager->setControlValue(mechanism, ControlModeControlName,    3); // MultiLoop control mode

        states.commandMode.state = nasa_r2_common_msgs::JointControlCommandMode::MULTILOOPSTEP;
        return;
    }
    RCS::Logger::getCategory("gov.nasa.JointControlCommandFsmGripper") << log4cpp::Priority::WARN << "Cannot transition to MULTILOOPSTEP, coeffs not loaded on joint: " << mechanism;
    RCS::Logger::getCategory("gov.nasa.JointControlCommandFsmGripper") << log4cpp::Priority::DEBUG << "Coeffs not loaded, transition to MULTILOOPSTEP not allowed on joint: " << mechanism;
}

/***************************************************************************//**
 *
 * @brief Transition command to JointControlMode::MULTILOOPSMOOTH for the joint
 *
 ******************************************************************************/
void JointControlCommandFsmGripper::multiLoopSmooth(void)
{
    RCS::Logger::getCategory("gov.nasa.JointControlCommandFsmGripper") << log4cpp::Priority::INFO << "MULTILOOPSMOOTH not defined for Grippers, trying MULTILOOPSTEP instead on joint: " << mechanism;
    multiLoopStep();
}

void JointControlCommandFsmGripper::actuator(void)
{
    RCS::Logger::getCategory("gov.nasa.JointControlCommandFsmGripper") << log4cpp::Priority::WARN << "ACTUATOR is not a valid JointControlCommandMode for " << mechanism;
}

// calibrationMode functions
/***************************************************************************//**
 *
 * @brief Transition command to JointControlCalibrationMode::DISABLE for the joint
 *
 ******************************************************************************/
void JointControlCommandFsmGripper::disableCalibrationMode(void)
{
    RCS::Logger::getCategory("gov.nasa.JointControlCommandFsmGripper") << log4cpp::Priority::DEBUG << "Command transition to JointControlCalibrationMode::DISABLE successful on joint: " << mechanism;
    nodeRegisterManager->setControlValue(mechanism, CalibrationModeControlName, 0);
    states.calibrationMode.state = nasa_r2_common_msgs::JointControlCalibrationMode::DISABLE;
}
/***************************************************************************//**
 *
 * @brief Transition command to JointControlCalibrationMode::ENABLE for the joint
 *
 ******************************************************************************/
void JointControlCommandFsmGripper::enableCalibrationMode(void)
{
    RCS::Logger::getCategory("gov.nasa.JointControlCommandFsmGripper") << log4cpp::Priority::DEBUG << "Command transition to JointControlCalibrationMode::ENABLE successful on joint: " << mechanism;
    nodeRegisterManager->setControlValue(mechanism, CalibrationModeControlName, 1);
    states.calibrationMode.state = nasa_r2_common_msgs::JointControlCalibrationMode::ENABLE;
}

// clearFaultMode functions
/***************************************************************************//**
 *
 * @brief Transition command to JointControlClearFaultMode::DISABLE for the joint
 *
 ******************************************************************************/
void JointControlCommandFsmGripper::disableClearFaultMode(void)
{
    RCS::Logger::getCategory("gov.nasa.JointControlCommandFsmGripper") << log4cpp::Priority::DEBUG << "Command transition to JointControlClearFaultMode::DISABLE successful on joint: " << mechanism;
    nodeRegisterManager->setControlValue(mechanism, ClearFaultControlName, 0);
    states.clearFaultMode.state = nasa_r2_common_msgs::JointControlClearFaultMode::DISABLE;
}
/***************************************************************************//**
 *
 * @brief Transition command to JointControlClearFaultMode::ENABLE for the joint
 *
 ******************************************************************************/
void JointControlCommandFsmGripper::enableClearFaultMode(void)
{
    RCS::Logger::getCategory("gov.nasa.JointControlCommandFsmGripper") << log4cpp::Priority::DEBUG << "Command transition to JointControlClearFaultMode::ENABLE successful on joint: " << mechanism;
    nodeRegisterManager->setControlValue(mechanism, ClearFaultControlName, 1);
    states.clearFaultMode.state = nasa_r2_common_msgs::JointControlClearFaultMode::ENABLE;
}
