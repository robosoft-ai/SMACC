/**
 * @file JointControlCommandFsmFinger.cpp
 */

#include "nasa_robodyn_mechanisms_core/JointControlCommandFsmFinger.h"
/***************************************************************************//**
 *
 * @brief Constructor for JointControlCommandFsmFinger
 *
 ******************************************************************************/
JointControlCommandFsmFinger::JointControlCommandFsmFinger(const std::string& mechanism, IoFunctions ioFunctions)
    : JointControlCommandFsmInterface(mechanism, ioFunctions)
{
    setParameters();

    bootLoader();
    motCom();
    enableCalibrationMode();
    disableClearFaultMode();
}

JointControlCommandFsmFinger::~JointControlCommandFsmFinger()
{
    // Nothing
}

void JointControlCommandFsmFinger::setParameters()
{
    std::string parameterFile = io.getControlFile(mechanism);

    //! Parse parameter file
    TiXmlDocument file(parameterFile.c_str());
    bool loadOkay = file.LoadFile();
    if (!loadOkay)
    {
        std::stringstream err;
        err << "Failed to load file [" << parameterFile << "]";
        RCS::Logger::log("gov.nasa.JointControlCommandFsmFinger", log4cpp::Priority::FATAL, err.str());
        throw std::runtime_error(err.str());
    }
    TiXmlHandle doc(&file);
    RCS::Logger::log("gov.nasa.JointControlCommandFsmFinger", log4cpp::Priority::INFO, "File [" + parameterFile + "] successfully loaded.");

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
        // Live coeffs names
        IsCalibratedLiveCoeffName = StringUtilities::makeFullyQualifiedRoboDynElement(mechanism, ApiMap::getXmlElementValue(parametersElement, "IsCalibratedLiveCoeff"));
    }
    else
    {
        std::stringstream err;
        err << "The file " << parameterFile << " has no element named [ApiMap]";
        RCS::Logger::log("gov.nasa.JointControlCommandFsmFinger", log4cpp::Priority::ERROR, err.str());
        throw std::runtime_error(err.str());
    }
}

/***************************************************************************//**
 *
 * @brief Retrieve state information
 * @return The state information
 *
 ******************************************************************************/
nasa_r2_common_msgs::JointControlData JointControlCommandFsmFinger::getStates(void)
{
    return states;
}

// controlMode functions
/***************************************************************************//**
 *
 * @brief Transition command to JointControlMode::BOOTLOADER for the joint
 *
 ******************************************************************************/
void JointControlCommandFsmFinger::bootLoader(void)
{
    RCS::Logger::getCategory("gov.nasa.JointControlCommandFsmFinger") << log4cpp::Priority::DEBUG << "Command transition to JointControlMode::BOOTLOADER successful on joint: " << mechanism;
    states.controlMode.state = nasa_r2_common_msgs::JointControlMode::BOOTLOADER;
}
/***************************************************************************//**
 *
 * @brief Transition command to JointControlMode::OFF for the joint
 *
 ******************************************************************************/
void JointControlCommandFsmFinger::off(void)
{
    RCS::Logger::getCategory("gov.nasa.JointControlCommandFsmFinger") << log4cpp::Priority::DEBUG << "Command transition to JointControlMode::OFF successful on joint: " << mechanism;
    states.controlMode.state = nasa_r2_common_msgs::JointControlMode::OFF;
}
/***************************************************************************//**
 *
 * @brief Transition command to JointControlMode::PARK for the joint
 *
 ******************************************************************************/
void JointControlCommandFsmFinger::park(void)
{
    RCS::Logger::getCategory("gov.nasa.JointControlCommandFsmFinger") << log4cpp::Priority::DEBUG << "Command transition to JointControlMode::PARK successful on joint: " << mechanism;
    states.controlMode.state = nasa_r2_common_msgs::JointControlMode::PARK;
}
/***************************************************************************//**
 *
 * @brief Transition command to JointControlMode::NEUTRAL for the joint
 *
 ******************************************************************************/
void JointControlCommandFsmFinger::neutral(void)
{
    RCS::Logger::getCategory("gov.nasa.JointControlCommandFsmFinger") << log4cpp::Priority::DEBUG << "Command transition to JointControlMode::NEUTRAL successful on joint: " << mechanism;
    states.controlMode.state = nasa_r2_common_msgs::JointControlMode::NEUTRAL;
}
/***************************************************************************//**
 *
 * @brief Transition command to JointControlMode::DRIVE for the joint
 *
 ******************************************************************************/
void JointControlCommandFsmFinger::drive(void)
{
    RCS::Logger::getCategory("gov.nasa.JointControlCommandFsmFinger") << log4cpp::Priority::DEBUG << "Command transition to JointControlMode::DRIVE successful on joint: " << mechanism;
    states.controlMode.state = nasa_r2_common_msgs::JointControlMode::DRIVE;
}

// commandMode functions
/***************************************************************************//**
 *
 * @brief Transition command to JointControlMode::MOTCOM for the joint
 *
 ******************************************************************************/
void JointControlCommandFsmFinger::motCom(void)
{
    RCS::Logger::getCategory("gov.nasa.JointControlCommandFsmFinger") << log4cpp::Priority::DEBUG << "Command transition to JointControlCommandMode::MOTCOM successful on mechanism: " << mechanism;
    states.commandMode.state = nasa_r2_common_msgs::JointControlCommandMode::MOTCOM;
}
/***************************************************************************//**
 *
 * @brief Transition command to JointControlCommandMode::STALLMODE for the joint
 *
 ******************************************************************************/
void JointControlCommandFsmFinger::stallMode(void)
{
    RCS::Logger::getCategory("gov.nasa.JointControlCommandFsmFinger") << log4cpp::Priority::DEBUG << "Command transition to JointControlCommandMode::STALLMODE successful on mechanism: " << mechanism;
    states.commandMode.state = nasa_r2_common_msgs::JointControlCommandMode::STALLMODE;
}
/***************************************************************************//**
 *
 * @brief Transition command to JointControlMode::MULTILOOPSTEP for the joint
 *
 ******************************************************************************/
void JointControlCommandFsmFinger::multiLoopStep(void)
{
    RCS::Logger::getCategory("gov.nasa.JointControlCommandFsmFinger") << log4cpp::Priority::DEBUG << "Command transition to JointControlCommandMode::MULTILOOPSTEP successful on mechanism: " << mechanism;
    states.commandMode.state = nasa_r2_common_msgs::JointControlCommandMode::MULTILOOPSTEP;
}
/***************************************************************************//**
 *
 * @brief Transition command to JointControlMode::MULTILOOPSMOOTH for the joint
 *
 ******************************************************************************/
void JointControlCommandFsmFinger::multiLoopSmooth(void)
{
    RCS::Logger::getCategory("gov.nasa.JointControlCommandFsmFinger") << log4cpp::Priority::DEBUG << "Command transition to JointControlCommandMode::MULTILOOPSMOOTH successful on mechanism: " << mechanism;
    states.commandMode.state = nasa_r2_common_msgs::JointControlCommandMode::MULTILOOPSMOOTH;
}

void JointControlCommandFsmFinger::actuator(void)
{
    RCS::Logger::getCategory("gov.nasa.JointControlCommandFsmFinger") << log4cpp::Priority::DEBUG << "Command transition to JointControlCommandMode::ACTUATOR successful on mechanism: " << mechanism;
    states.commandMode.state = nasa_r2_common_msgs::JointControlCommandMode::ACTUATOR;
}

// calibrationMode functions
/***************************************************************************//**
 *
 * @brief Transition command to JointControlCalibrationMode::DISABLE for the joint
 *
 ******************************************************************************/
void JointControlCommandFsmFinger::disableCalibrationMode(void)
{
    RCS::Logger::getCategory("gov.nasa.JointControlCommandFsmFinger") << log4cpp::Priority::DEBUG << "Command transition to JointControlCalibrationMode::DISABLE successful on mechanism: " << mechanism;
    states.calibrationMode.state = nasa_r2_common_msgs::JointControlCalibrationMode::DISABLE;
}
/***************************************************************************//**
 *
 * @brief Transition command to JointControlCalibrationMode::ENABLE for the joint
 *
 ******************************************************************************/
void JointControlCommandFsmFinger::enableCalibrationMode(void)
{
    io.setLiveCoeff(IsCalibratedLiveCoeffName, 0);
    RCS::Logger::getCategory("gov.nasa.JointControlCommandFsmFinger") << log4cpp::Priority::DEBUG << "Command transition to JointControlCalibrationMode::ENABLE successful on mechanism: " << mechanism;
    states.calibrationMode.state = nasa_r2_common_msgs::JointControlCalibrationMode::ENABLE;
}

// clearFaultMode functions
/***************************************************************************//**
 *
 * @brief Transition command to JointControlClearFaultMode::DISABLE for the joint
 *
 ******************************************************************************/
void JointControlCommandFsmFinger::disableClearFaultMode(void)
{
    RCS::Logger::getCategory("gov.nasa.JointControlCommandFsmFinger") << log4cpp::Priority::DEBUG << "Command transition to JointControlClearFaultMode::DISABLE successful on mechanism: " << mechanism;
    states.clearFaultMode.state = nasa_r2_common_msgs::JointControlClearFaultMode::DISABLE;
}
/***************************************************************************//**
 *
 * @brief Transition command to JointControlClearFaultMode::ENABLE for the joint
 *
 ******************************************************************************/
void JointControlCommandFsmFinger::enableClearFaultMode(void)
{
    RCS::Logger::getCategory("gov.nasa.JointControlCommandFsmFinger") << log4cpp::Priority::DEBUG << "Command transition to JointControlClearFaultMode::ENABLE successful on mechanism: " << mechanism;
    states.clearFaultMode.state = nasa_r2_common_msgs::JointControlClearFaultMode::ENABLE;
}
