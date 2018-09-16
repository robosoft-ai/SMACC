/**
 * @file JointControlActualFsmWrist.cpp
 */

#include "nasa_robodyn_mechanisms_core/JointControlActualFsmWrist.h"
/***************************************************************************//**
 *
 * @brief Constructor for JointControlActualFsmWrist
 *
 * @param mechanism
 * @param ioFunctions
 * @throw invalid_argument if any of the io file is empty
 *
 ******************************************************************************/
JointControlActualFsmWrist::JointControlActualFsmWrist(const std::string& mechanism, IoFunctions ioFunctions)
    : JointControlActualFsmInterface(mechanism, ioFunctions)
{
    if( io.hasLiveCoeff.empty()    or
        io.getLiveCoeff.empty()    or
        io.setLiveCoeff.empty()    )
    {
        std::stringstream err;
        err << "Constructor requires 'io.hasLiveCoeff', 'io.getLiveCoeff', 'io.setLiveCoeff' be non-empty.";
        RCS::Logger::log("gov.nasa.JointControlActualFsmWrist", log4cpp::Priority::FATAL, err.str());
        throw std::invalid_argument(err.str());
    }

    setParameters();

    // Set up expected values @bootup
    io.setLiveCoeff(ControlModeLiveCoeffName,     nasa_r2_common_msgs::JointControlMode::BOOTLOADER);
    io.setLiveCoeff(CommandModeLiveCoeffName,     nasa_r2_common_msgs::JointControlCommandMode::INVALID);
    io.setLiveCoeff(CalibrationModeLiveCoeffName, nasa_r2_common_msgs::JointControlCalibrationMode::IGNORE);
    io.setLiveCoeff(CoeffStateLiveCoeffName,      nasa_r2_common_msgs::JointControlCoeffState::LOADED);
}

JointControlActualFsmWrist::~JointControlActualFsmWrist()
{
    // Nothing
}

void JointControlActualFsmWrist::setParameters()
{
    std::string parameterFile = io.getControlFile(mechanism);

    //! Parse parameter file
    TiXmlDocument file(parameterFile.c_str());
    bool loadOkay = file.LoadFile();
    if (!loadOkay)
    {
        std::stringstream err;
        err << "Failed to load file [" << parameterFile << "]";
        RCS::Logger::log("gov.nasa.JointControlActualFsmWrist", log4cpp::Priority::FATAL, err.str());
        throw std::runtime_error(err.str());
    }
    TiXmlHandle doc(&file);
    RCS::Logger::log("gov.nasa.JointControlActualFsmWrist", log4cpp::Priority::INFO, "File [" + parameterFile + "] successfully loaded.");

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
        PitchLimitLiveCoeffName            = StringUtilities::makeFullyQualifiedRoboDynElement(mechanism, ApiMap::getXmlElementValue(parametersElement, "PitchLimitLiveCoeff"));
        YawLimitLiveCoeffName              = StringUtilities::makeFullyQualifiedRoboDynElement(mechanism, ApiMap::getXmlElementValue(parametersElement, "YawLimitLiveCoeff"));
        LittlesideSliderLimitLiveCoeffName = StringUtilities::makeFullyQualifiedRoboDynElement(mechanism, ApiMap::getXmlElementValue(parametersElement, "LittlesideSliderLimitLiveCoeff"));
        ThumbsideSliderLimitLiveCoeffName  = StringUtilities::makeFullyQualifiedRoboDynElement(mechanism, ApiMap::getXmlElementValue(parametersElement, "ThumbsideSliderLimitLiveCoeff"));
        SensorErrorLiveCoeffName           = StringUtilities::makeFullyQualifiedRoboDynElement(mechanism, ApiMap::getXmlElementValue(parametersElement, "SensorErrorLiveCoeff"));
        SliderDiffErrorLiveCoeffName       = StringUtilities::makeFullyQualifiedRoboDynElement(mechanism, ApiMap::getXmlElementValue(parametersElement, "SliderDiffErrorLiveCoeff"));
        ControlModeLiveCoeffName           = StringUtilities::makeFullyQualifiedRoboDynElement(mechanism, ApiMap::getXmlElementValue(parametersElement, "ControlModeLiveCoeff"));
        CommandModeLiveCoeffName           = StringUtilities::makeFullyQualifiedRoboDynElement(mechanism, ApiMap::getXmlElementValue(parametersElement, "CommandModeLiveCoeff"));
        CalibrationModeLiveCoeffName       = StringUtilities::makeFullyQualifiedRoboDynElement(mechanism, ApiMap::getXmlElementValue(parametersElement, "CalibrationModeLiveCoeff"));
        CoeffStateLiveCoeffName            = StringUtilities::makeFullyQualifiedRoboDynElement(mechanism, ApiMap::getXmlElementValue(parametersElement, "CoeffStateLiveCoeff"));
    }
    else
    {
        std::stringstream err;
        err << "The file " << parameterFile << " has no element named [ApiMap]";
        RCS::Logger::log("gov.nasa.JointControlActualFsmWrist", log4cpp::Priority::ERROR, err.str());
        throw std::runtime_error(err.str());
    }
}

/***************************************************************************//**
 *
 * @brief get the Command, Control, and Calibration Mode states, as well as the Coeff State
 * @return The state information
 *
 ******************************************************************************/
nasa_r2_common_msgs::JointControlData JointControlActualFsmWrist::getStates(void)
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
void JointControlActualFsmWrist::updateControlModeState(void)
{

    // Verify existence of live coeffs
    if(not(io.hasLiveCoeff(ControlModeLiveCoeffName)))
    {
        RCS::Logger::getCategory("gov.nasa.JointControlActualFsmWrist") << log4cpp::Priority::ERROR << mechanism << ": Missing live coeff: " << ControlModeLiveCoeffName;
        return;
    }
    if(not(io.hasLiveCoeff(PitchLimitLiveCoeffName)))
    {
        RCS::Logger::getCategory("gov.nasa.JointControlActualFsmWrist") << log4cpp::Priority::ERROR << mechanism << ": Missing live coeff: " << PitchLimitLiveCoeffName;
        return;
    }
    if(not(io.hasLiveCoeff(YawLimitLiveCoeffName)))
    {
        RCS::Logger::getCategory("gov.nasa.JointControlActualFsmWrist") << log4cpp::Priority::ERROR << mechanism << ": Missing live coeff: " << YawLimitLiveCoeffName;
        return;
    }
    if(not(io.hasLiveCoeff(LittlesideSliderLimitLiveCoeffName)))
    {
        RCS::Logger::getCategory("gov.nasa.JointControlActualFsmWrist") << log4cpp::Priority::ERROR << mechanism << ": Missing live coeff: " << LittlesideSliderLimitLiveCoeffName;
        return;
    }
    if(not(io.hasLiveCoeff(ThumbsideSliderLimitLiveCoeffName)))
    {
        RCS::Logger::getCategory("gov.nasa.JointControlActualFsmWrist") << log4cpp::Priority::ERROR << mechanism << ": Missing live coeff: " << ThumbsideSliderLimitLiveCoeffName;
        return;
    }
    if(not(io.hasLiveCoeff(SensorErrorLiveCoeffName)))
    {
        RCS::Logger::getCategory("gov.nasa.JointControlActualFsmWrist") << log4cpp::Priority::ERROR << mechanism << ": Missing live coeff: " << SensorErrorLiveCoeffName;
        return;
    }
    if(not(io.hasLiveCoeff(SliderDiffErrorLiveCoeffName)))
    {
        RCS::Logger::getCategory("gov.nasa.JointControlActualFsmWrist") << log4cpp::Priority::ERROR << mechanism << ": Missing live coeff: " << SliderDiffErrorLiveCoeffName;
        return;
    }

    // Check for FAULTED state
    if( (io.getLiveCoeff(PitchLimitLiveCoeffName)            == 1) ||
        (io.getLiveCoeff(YawLimitLiveCoeffName)              == 1) ||
        (io.getLiveCoeff(LittlesideSliderLimitLiveCoeffName) == 1) ||
        (io.getLiveCoeff(ThumbsideSliderLimitLiveCoeffName)  == 1) ||
        (io.getLiveCoeff(SensorErrorLiveCoeffName)           == 1) ||
        (io.getLiveCoeff(SliderDiffErrorLiveCoeffName)       == 1) )
    {
        if(states.controlMode.state != nasa_r2_common_msgs::JointControlMode::FAULTED)
        {
            // Output debugging messages
            if(io.getLiveCoeff(PitchLimitLiveCoeffName)            == 1)
            {
                RCS::Logger::getCategory("gov.nasa.JointControlActualFsmWrist") << log4cpp::Priority::WARN << mechanism << ": PITCH FAULT DETECTED";
            }
            if(io.getLiveCoeff(YawLimitLiveCoeffName)              == 1)
            {
                RCS::Logger::getCategory("gov.nasa.JointControlActualFsmWrist") << log4cpp::Priority::WARN << mechanism << ": YAW FAULT DETECTED";
            }
            if(io.getLiveCoeff(LittlesideSliderLimitLiveCoeffName) == 1)
            {
                RCS::Logger::getCategory("gov.nasa.JointControlActualFsmWrist") << log4cpp::Priority::WARN << mechanism << ": LITTLESIDE SLIDER FAULT DETECTED";
            }
            if(io.getLiveCoeff(ThumbsideSliderLimitLiveCoeffName)  == 1)
            {
                RCS::Logger::getCategory("gov.nasa.JointControlActualFsmWrist") << log4cpp::Priority::WARN << mechanism << ": THUMBSIDE SLIDER FAULT DETECTED";
            }
            if(io.getLiveCoeff(SensorErrorLiveCoeffName)           == 1)
            {
                RCS::Logger::getCategory("gov.nasa.JointControlActualFsmWrist") << log4cpp::Priority::WARN << mechanism << ": SENSOR FAULT DETECTED";
            }
            if(io.getLiveCoeff(SliderDiffErrorLiveCoeffName)       == 1)
            {
                RCS::Logger::getCategory("gov.nasa.JointControlActualFsmWrist") << log4cpp::Priority::WARN << mechanism << ": SLIDER DIFF FAULT DETECTED";
            }
            RCS::Logger::getCategory("gov.nasa.JointControlActualFsmWrist") << log4cpp::Priority::DEBUG << "Transition to JointControlMode::FAULTED on joint: " << mechanism;
            states.controlMode.state = nasa_r2_common_msgs::JointControlMode::FAULTED;
        }
        return;
    }

    states.controlMode.state = io.getLiveCoeff(ControlModeLiveCoeffName);

}
/***************************************************************************//**
 *
 * @brief Update the command mode state 
 *
 ******************************************************************************/
void JointControlActualFsmWrist::updateCommandModeState(void)
{
    // Verify existence of live coeffs
    if(not(io.hasLiveCoeff(CommandModeLiveCoeffName)))
    {
        RCS::Logger::getCategory("gov.nasa.JointControlActualFsmWrist") << log4cpp::Priority::ERROR << mechanism << ": Missing live coeff: " << CommandModeLiveCoeffName;
        return;
    }

    states.commandMode.state = io.getLiveCoeff(CommandModeLiveCoeffName);
}
/***************************************************************************//**
 *
 * @brief Update the calibration mode state 
 *
 ******************************************************************************/
void JointControlActualFsmWrist::updateCalibrationModeState(void)
{
    if(not(io.hasLiveCoeff(CalibrationModeLiveCoeffName)))
    {
        RCS::Logger::getCategory("gov.nasa.JointControlActualFsmWrist") << log4cpp::Priority::ERROR << mechanism << ": Missing live coeff: " << CalibrationModeLiveCoeffName;
        return;
    }

    states.calibrationMode.state = io.getLiveCoeff(CalibrationModeLiveCoeffName);
}
/***************************************************************************//**
 *
 * @brief Update the clearFault mode state 
 *
 ******************************************************************************/
void JointControlActualFsmWrist::updateClearFaultModeState(void)
{
    // Handled by JointControlManager
    states.clearFaultMode.state = nasa_r2_common_msgs::JointControlClearFaultMode::DISABLE;
}
/***************************************************************************//**
 *
 * @brief Update the coefficient state 
 *
 ******************************************************************************/
void JointControlActualFsmWrist::updateCoeffState(void)
{
    // Verify existence of live coeffs
    if(not(io.hasLiveCoeff(CoeffStateLiveCoeffName)))
    {
        RCS::Logger::getCategory("gov.nasa.JointControlActualFsmWrist") << log4cpp::Priority::ERROR << mechanism << ": Missing live coeff: " << CoeffStateLiveCoeffName;
        return;
    }

    states.coeffState.state = io.getLiveCoeff(CoeffStateLiveCoeffName);
}
