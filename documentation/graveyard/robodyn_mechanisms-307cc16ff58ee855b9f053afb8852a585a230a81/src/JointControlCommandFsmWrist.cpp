/**
 * @file JointControlCommandFsmWrist.cpp
 */

#include "nasa_robodyn_mechanisms_core/JointControlCommandFsmWrist.h"
/***************************************************************************//**
 *
 * @brief Constructor for JointControlCommandFsmWrist
 *
 ******************************************************************************/
JointControlCommandFsmWrist::JointControlCommandFsmWrist(const std::string& mechanism, IoFunctions ioFunctions)
    : JointControlCommandFsmInterface(mechanism, ioFunctions)
{
    if( io.setLiveCoeff.empty() )
    {
        std::stringstream err;
        err << "Constructor requires 'io.setLiveCoeff' be non-empty.";
        RCS::Logger::log("gov.nasa.JointControlActualFsmWrist", log4cpp::Priority::FATAL, err.str());
        throw std::invalid_argument(err.str());
    }

    setParameters();

    bootLoader();
    multiLoopStep();
    states.calibrationMode.state = nasa_r2_common_msgs::JointControlCalibrationMode::IGNORE;
    disableClearFaultMode();
}

JointControlCommandFsmWrist::~JointControlCommandFsmWrist()
{
    // Nothing
}
void JointControlCommandFsmWrist::setParameters()
{
    std::string parameterFile = io.getControlFile(mechanism);

    //! Parse parameter file
    TiXmlDocument file(parameterFile.c_str());
    bool loadOkay = file.LoadFile();
    if (!loadOkay)
    {
        std::stringstream err;
        err << "Failed to load file [" << parameterFile << "]";
        RCS::Logger::log("gov.nasa.JointControlCommandFsmWrist", log4cpp::Priority::FATAL, err.str());
        throw std::runtime_error(err.str());
    }
    TiXmlHandle doc(&file);
    RCS::Logger::log("gov.nasa.JointControlCommandFsmWrist", log4cpp::Priority::INFO, "File [" + parameterFile + "] successfully loaded.");

    // Print the XML file's contents
    //cout << "Successfully loaded file: " << parameterFile << endl;
    //TiXmlPrinter printer;
    //printer.SetIndent("\t");
    //file.Accept(&printer);
    //cout << printer.Str() << endl;

    // Check for ApiMap
    TiXmlHandle parametersElement(doc.FirstChildElement("ApiMap"));

    if (parametersElement.ToElement())
    {   // Live coeffs names
        ControlModeLiveCoeffName           = StringUtilities::makeFullyQualifiedRoboDynElement(mechanism, ApiMap::getXmlElementValue(parametersElement, "ControlModeLiveCoeff"));
        CommandModeLiveCoeffName           = StringUtilities::makeFullyQualifiedRoboDynElement(mechanism, ApiMap::getXmlElementValue(parametersElement, "CommandModeLiveCoeff"));
        CalibrationModeLiveCoeffName       = StringUtilities::makeFullyQualifiedRoboDynElement(mechanism, ApiMap::getXmlElementValue(parametersElement, "CalibrationModeLiveCoeff"));
    }
    else
    {
        std::stringstream err;
        err << "The file " << parameterFile << " has no element named [ApiMap]";
        RCS::Logger::log("gov.nasa.JointControlCommandFsmWrist", log4cpp::Priority::ERROR, err.str());
        throw std::runtime_error(err.str());
    }
}

/***************************************************************************//**
 *
 * @brief Retrieve state information
 * @return The state information
 *
 ******************************************************************************/
nasa_r2_common_msgs::JointControlData JointControlCommandFsmWrist::getStates(void)
{
    return states;
}

// controlMode functions
/***************************************************************************//**
 *
 * @brief Transition command to JointControlMode::BOOTLOADER for the joint
 *
 ******************************************************************************/
void JointControlCommandFsmWrist::bootLoader(void)
{
    RCS::Logger::getCategory("gov.nasa.JointControlCommandFsmWrist") << log4cpp::Priority::DEBUG << "Command transition to JointControlMode::BOOTLOADER successful on mechanism: " << mechanism << ". Commanding Park.";
    states.controlMode.state = nasa_r2_common_msgs::JointControlMode::BOOTLOADER;
    io.setLiveCoeff(ControlModeLiveCoeffName, nasa_r2_common_msgs::JointControlMode::BOOTLOADER);
}
/***************************************************************************//**
 *
 * @brief Transition command to JointControlMode::OFF for the joint
 *
 ******************************************************************************/
void JointControlCommandFsmWrist::off(void)
{
    RCS::Logger::getCategory("gov.nasa.JointControlCommandFsmWrist") << log4cpp::Priority::DEBUG << "Command transition to JointControlMode::OFF ignored on mechanism: " << mechanism;
    states.controlMode.state = nasa_r2_common_msgs::JointControlMode::OFF;
    io.setLiveCoeff(ControlModeLiveCoeffName, nasa_r2_common_msgs::JointControlMode::OFF);
}
/***************************************************************************//**
 *
 * @brief Transition command to JointControlMode::PARK for the joint
 *
 ******************************************************************************/
void JointControlCommandFsmWrist::park(void)
{
    RCS::Logger::getCategory("gov.nasa.JointControlCommandFsmWrist") << log4cpp::Priority::DEBUG << "Command transition to JointControlMode::PARK successful on mechanism: " << mechanism;
    states.controlMode.state = nasa_r2_common_msgs::JointControlMode::PARK;
    io.setLiveCoeff(ControlModeLiveCoeffName, nasa_r2_common_msgs::JointControlMode::PARK);

}
/***************************************************************************//**
 *
 * @brief Transition command to JointControlMode::NEUTRAL for the joint
 *
 ******************************************************************************/
void JointControlCommandFsmWrist::neutral(void)
{
    RCS::Logger::getCategory("gov.nasa.JointControlCommandFsmWrist") << log4cpp::Priority::DEBUG << "Command transition to JointControlMode::NEUTRAL ignored on mechanism: " << mechanism;
    states.controlMode.state = nasa_r2_common_msgs::JointControlMode::NEUTRAL;
    io.setLiveCoeff(ControlModeLiveCoeffName, nasa_r2_common_msgs::JointControlMode::NEUTRAL);
}
/***************************************************************************//**
 *
 * @brief Transition command to JointControlMode::DRIVE for the joint
 *
 ******************************************************************************/
void JointControlCommandFsmWrist::drive(void)
{
    RCS::Logger::getCategory("gov.nasa.JointControlCommandFsmWrist") << log4cpp::Priority::DEBUG << "Command transition to JointControlMode::DRIVE successful on mechanism: " << mechanism;
    states.controlMode.state = nasa_r2_common_msgs::JointControlMode::DRIVE;
    io.setLiveCoeff(ControlModeLiveCoeffName, nasa_r2_common_msgs::JointControlMode::DRIVE);

}

// commandMode functions
/***************************************************************************//**
 *
 * @brief Transition command to JointControlMode::MOTCOM for the joint
 *
 ******************************************************************************/
void JointControlCommandFsmWrist::motCom(void)
{
    RCS::Logger::getCategory("gov.nasa.JointControlCommandFsmWrist") << log4cpp::Priority::DEBUG << "Command transition to JointControlCommandMode::MOTCOM ignored on mechanism: " << mechanism;
    states.commandMode.state = nasa_r2_common_msgs::JointControlCommandMode::MOTCOM;
    io.setLiveCoeff(CommandModeLiveCoeffName, nasa_r2_common_msgs::JointControlCommandMode::MOTCOM);
}
/***************************************************************************//**
 *
 * @brief Transition command to JointControlCommandMode::STALLMODE for the joint
 *
 ******************************************************************************/
void JointControlCommandFsmWrist::stallMode(void)
{
    RCS::Logger::getCategory("gov.nasa.JointControlCommandFsmWrist") << log4cpp::Priority::DEBUG << "Command transition to JointControlCommandMode::STALLMODE successful on mechanism: " << mechanism;
    states.commandMode.state = nasa_r2_common_msgs::JointControlCommandMode::STALLMODE;
}
/***************************************************************************//**
 *
 * @brief Transition command to JointControlMode::MULTILOOPSTEP for the joint
 *
 ******************************************************************************/
void JointControlCommandFsmWrist::multiLoopStep(void)
{
    RCS::Logger::getCategory("gov.nasa.JointControlCommandFsmWrist") << log4cpp::Priority::DEBUG << "Command transition to JointControlCommandMode::MULTILOOPSTEP successful on mechanism: " << mechanism;
    states.commandMode.state = nasa_r2_common_msgs::JointControlCommandMode::MULTILOOPSTEP;
    io.setLiveCoeff(CommandModeLiveCoeffName, nasa_r2_common_msgs::JointControlCommandMode::MULTILOOPSTEP);

}
/***************************************************************************//**
 *
 * @brief Transition command to JointControlMode::MULTILOOPSMOOTH for the joint
 *
 ******************************************************************************/
void JointControlCommandFsmWrist::multiLoopSmooth(void)
{
    RCS::Logger::getCategory("gov.nasa.JointControlCommandFsmWrist") << log4cpp::Priority::DEBUG << "Command transition to JointControlCommandMode::MULTILOOPSMOOTH ignored on mechanism: " << mechanism;
    states.commandMode.state = nasa_r2_common_msgs::JointControlCommandMode::MULTILOOPSMOOTH;
    io.setLiveCoeff(CommandModeLiveCoeffName, nasa_r2_common_msgs::JointControlCommandMode::MULTILOOPSMOOTH);

}

void JointControlCommandFsmWrist::actuator(void)
{
    RCS::Logger::getCategory("gov.nasa.JointControlCommandFsmWrist") << log4cpp::Priority::DEBUG << "Command transition to JointControlCommandMode::ACTUATOR successful on mechanism: " << mechanism;
    states.commandMode.state = nasa_r2_common_msgs::JointControlCommandMode::ACTUATOR;
    io.setLiveCoeff(CommandModeLiveCoeffName, nasa_r2_common_msgs::JointControlCommandMode::ACTUATOR);

}

// calibrationMode functions
/***************************************************************************//**
 *
 * @brief Transition command to JointControlCalibrationMode::DISABLE for the joint
 *
 ******************************************************************************/
void JointControlCommandFsmWrist::disableCalibrationMode(void)
{
    RCS::Logger::getCategory("gov.nasa.JointControlCommandFsmWrist") << log4cpp::Priority::DEBUG << "Command transition to JointControlCalibrationMode::DISABLE successful on mechanism: " << mechanism;
    states.calibrationMode.state = nasa_r2_common_msgs::JointControlCalibrationMode::DISABLE;
}
/***************************************************************************//**
 *
 * @brief Transition command to JointControlCalibrationMode::ENABLE for the joint
 *
 ******************************************************************************/
void JointControlCommandFsmWrist::enableCalibrationMode(void)
{
    RCS::Logger::getCategory("gov.nasa.JointControlCommandFsmWrist") << log4cpp::Priority::DEBUG << "Command transition to JointControlCalibrationMode::ENABLE successful on mechanism: " << mechanism;
    io.setLiveCoeff(CalibrationModeLiveCoeffName, nasa_r2_common_msgs::JointControlCalibrationMode::ENABLE);
    states.calibrationMode.state = nasa_r2_common_msgs::JointControlCalibrationMode::ENABLE;

}

// clearFaultMode functions
/***************************************************************************//**
 *
 * @brief Transition command to JointControlClearFaultMode::DISABLE for the joint
 *
 ******************************************************************************/
void JointControlCommandFsmWrist::disableClearFaultMode(void)
{
    RCS::Logger::getCategory("gov.nasa.JointControlCommandFsmWrist") << log4cpp::Priority::DEBUG << "Command transition to JointControlClearFaultMode::DISABLE successful on mechanism: " << mechanism;
    states.clearFaultMode.state = nasa_r2_common_msgs::JointControlClearFaultMode::DISABLE;
}
/***************************************************************************//**
 *
 * @brief Transition command to JointControlClearFaultMode::ENABLE for the joint
 *
 ******************************************************************************/
void JointControlCommandFsmWrist::enableClearFaultMode(void)
{
    RCS::Logger::getCategory("gov.nasa.JointControlCommandFsmWrist") << log4cpp::Priority::DEBUG << "Command transition to JointControlClearFaultMode::ENABLE successful on mechanism: " << mechanism;
    states.clearFaultMode.state = nasa_r2_common_msgs::JointControlClearFaultMode::ENABLE;
}
