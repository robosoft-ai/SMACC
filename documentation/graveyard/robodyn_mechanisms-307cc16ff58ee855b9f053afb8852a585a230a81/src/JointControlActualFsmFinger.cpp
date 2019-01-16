/**
 * @file JointControlActualFsmFinger.cpp
 */

#include "nasa_robodyn_mechanisms_core/JointControlActualFsmFinger.h"
/***************************************************************************//**
 *
 * @brief Constructor for JointControlActualFsmFinger
 *
 * @param mechanism
 * @param ioFunctions
 * @throw invalid_argument if any of the io file is empty
 *
 ******************************************************************************/
JointControlActualFsmFinger::JointControlActualFsmFinger(const std::string& mechanism, IoFunctions ioFunctions)
    : JointControlActualFsmInterface(mechanism, ioFunctions)
{
    if( io.hasLiveCoeff.empty()    or
        io.getLiveCoeff.empty()    or
        io.setLiveCoeff.empty()    )
    {
        std::stringstream err;
        err << "Constructor requires 'io.hasLiveCoeff', 'io.getLiveCoeff', 'io.setLiveCoeff' be non-empty.";
        RCS::Logger::log("gov.nasa.JointControlActualFsmFinger", log4cpp::Priority::FATAL, err.str());
        throw std::invalid_argument(err.str());
    }

    setParameters();
}

JointControlActualFsmFinger::~JointControlActualFsmFinger()
{
    // Nothing
}

void JointControlActualFsmFinger::setParameters()
{
    std::string parameterFile = io.getControlFile(mechanism);

    //! Parse parameter file
    TiXmlDocument file(parameterFile.c_str());
    bool loadOkay = file.LoadFile();
    if (!loadOkay)
    {
        std::stringstream err;
        err << "Failed to load file [" << parameterFile << "]";
        RCS::Logger::log("gov.nasa.JointControlActualFsmFinger", log4cpp::Priority::FATAL, err.str());
        throw std::runtime_error(err.str());
    }
    TiXmlHandle doc(&file);
    RCS::Logger::log("gov.nasa.JointControlActualFsmFinger", log4cpp::Priority::INFO, "File [" + parameterFile + "] successfully loaded.");

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
        RCS::Logger::log("gov.nasa.JointControlActualFsmFinger", log4cpp::Priority::ERROR, err.str());
        throw std::runtime_error(err.str());
    }
}

/***************************************************************************//**
 *
 * @brief get the Command, Control, and Calibration Mode states, as well as the Coeff State
 * @return The state information
 *
 ******************************************************************************/
nasa_r2_common_msgs::JointControlData JointControlActualFsmFinger::getStates(void)
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
void JointControlActualFsmFinger::updateControlModeState(void)
{
    // Verify existence of live coeffs
    if(not(io.hasLiveCoeff(IsCalibratedLiveCoeffName)))
    {
        RCS::Logger::getCategory("gov.nasa.JointControlActualFsmFinger") << log4cpp::Priority::ERROR << mechanism << ": Missing live coeff: " << IsCalibratedLiveCoeffName;
        return;
    }

    // The other states are handled in the JointControlManager for Fingers
    if(states.controlMode.state != nasa_r2_common_msgs::JointControlMode::PARK)
    {
        RCS::Logger::getCategory("gov.nasa.JointControlActualFsmFinger") << log4cpp::Priority::DEBUG << "Transition to JointControlMode::PARK on joint: " << mechanism;
        states.controlMode.state = nasa_r2_common_msgs::JointControlMode::PARK;
    }
    return;
}
/***************************************************************************//**
 *
 * @brief Update the command mode state 
 *
 ******************************************************************************/
void JointControlActualFsmFinger::updateCommandModeState(void)
{
    // Handled by JointControlManager
    states.commandMode.state = nasa_r2_common_msgs::JointControlCommandMode::INVALID;
}
/***************************************************************************//**
 *
 * @brief Update the calibration mode state 
 *
 ******************************************************************************/
void JointControlActualFsmFinger::updateCalibrationModeState(void)
{
    if(not(io.hasLiveCoeff(IsCalibratedLiveCoeffName)))
    {
        RCS::Logger::getCategory("gov.nasa.JointControlActualFsmWrist") << log4cpp::Priority::ERROR << mechanism << ": Missing live coeff: " << IsCalibratedLiveCoeffName;
        return;
    }

    // Check for DISABLE
    if(io.getLiveCoeff(IsCalibratedLiveCoeffName) == 1)
    {
        if(states.calibrationMode.state != nasa_r2_common_msgs::JointControlCalibrationMode::DISABLE)
        {
            RCS::Logger::getCategory("gov.nasa.JointControlActualFsmWrist") << log4cpp::Priority::DEBUG << "Transition to JointControlCalibrationMode::DISABLE on joint: " << mechanism;
            states.calibrationMode.state = nasa_r2_common_msgs::JointControlCalibrationMode::DISABLE;
        }
        return;
    }

    // If not DISABLE then ENABLE
    if(states.calibrationMode.state != nasa_r2_common_msgs::JointControlCalibrationMode::ENABLE)
    {
        RCS::Logger::getCategory("gov.nasa.JointControlActualFsmWrist") << log4cpp::Priority::DEBUG << "Transition to JointControlCalibrationMode::ENABLE on joint: " << mechanism;
        states.calibrationMode.state = nasa_r2_common_msgs::JointControlCalibrationMode::ENABLE;
    }
}
/***************************************************************************//**
 *
 * @brief Update the clearFault mode state 
 *
 ******************************************************************************/
void JointControlActualFsmFinger::updateClearFaultModeState(void)
{
    // Handled by JointControlManager
    states.clearFaultMode.state = nasa_r2_common_msgs::JointControlClearFaultMode::DISABLE;
}
/***************************************************************************//**
 *
 * @brief Update the coefficient state 
 *
 ******************************************************************************/
void JointControlActualFsmFinger::updateCoeffState(void)
{
    // Handled by JointControlManager
    states.coeffState.state = nasa_r2_common_msgs::JointControlCoeffState::LOADED;
}
