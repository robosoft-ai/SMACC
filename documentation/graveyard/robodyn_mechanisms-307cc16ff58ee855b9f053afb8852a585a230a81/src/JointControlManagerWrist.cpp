/**
 * @file JointControlManagerWrist.cpp
 */

#include "nasa_robodyn_mechanisms_core/JointControlManagerWrist.h"

JointControlManagerWrist::JointControlManagerWrist(const std::string& mechanism, IoFunctions ioFunctions, double timeLimit, const std::string& type)
    : JointControlManagerInterface(mechanism, ioFunctions, timeLimit, type)
{
    if(io.getLiveCoeff.empty())
    {
        std::stringstream err;
        err << "Constructor requires 'io.getLiveCoeff' be non-empty.";
        RCS::Logger::log("gov.nasa.JointControlActualFsmWrist", log4cpp::Priority::FATAL, err.str());
        throw std::invalid_argument(err.str());
    }

    // Initialize variables
    controlModeBadStart     = ros::Time::now();
    commandModeBadStart     = ros::Time::now();
    calibrationModeBadStart = ros::Time::now();

    // Instantiate state machines
    actualFsm  = boost::make_shared<JointControlActualFsmWrist> (mechanism, ioFunctions);
    commandFsm = boost::make_shared<JointControlCommandFsmWrist>(mechanism, ioFunctions);
}

JointControlManagerWrist::~JointControlManagerWrist()
{
    // Nothing
}

void JointControlManagerWrist::setParameters()
{
    std::string parameterFile = io.getControlFile(mechanism);

    //! Parse parameter file
    TiXmlDocument file(parameterFile.c_str());
    bool loadOkay = file.LoadFile();
    if (!loadOkay)
    {
        std::stringstream err;
        err << "Failed to load file [" << parameterFile << "]";
        RCS::Logger::log("gov.nasa.JointControlManagerWrist", log4cpp::Priority::FATAL, err.str());
        throw std::runtime_error(err.str());
    }
    TiXmlHandle doc(&file);
    RCS::Logger::log("gov.nasa.JointControlManagerWrist", log4cpp::Priority::INFO, "File [" + parameterFile + "] successfully loaded.");

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
    }
    else
    {
        std::stringstream err;
        err << "The file " << parameterFile << " has no element named [ApiMap]";
        RCS::Logger::log("gov.nasa.JointControlManagerWrist", log4cpp::Priority::ERROR, err.str());
        throw std::runtime_error(err.str());
    }
}

nasa_r2_common_msgs::JointControlData JointControlManagerWrist::getActualStates(void)
{
    nasa_r2_common_msgs::JointControlData actualStatesLocal  = actualFsm->getStates();
    nasa_r2_common_msgs::JointControlData commandStatesLocal = commandFsm->getStates();

    //! Here is where the current actual state should update the commanded state
    switch(actualStatesLocal.controlMode.state)
    {
        case nasa_r2_common_msgs::JointControlMode::BOOTLOADER:
            if(actualStatesLocal.calibrationMode.state == nasa_r2_common_msgs::JointControlCalibrationMode::ENABLE)
            {
                RCS::Logger::getCategory("gov.nasa.JointControlActualFsmWrist") << log4cpp::Priority::DEBUG << "Transition to JointControlMode::PARK on joint: " << mechanism;
                actualStatesLocal.controlMode.state = nasa_r2_common_msgs::JointControlMode::PARK;
                commandFsm->park();
            }
            break;
        case nasa_r2_common_msgs::JointControlMode::DRIVE:
            if(actualStatesLocal.calibrationMode.state != nasa_r2_common_msgs::JointControlCalibrationMode::DISABLE)
            {
                RCS::Logger::getCategory("gov.nasa.JointControlActualFsmWrist") << log4cpp::Priority::WARN << "Cannot calibrate in DRIVE on " << mechanism <<"; setting to park()";
                actualStatesLocal.controlMode.state = nasa_r2_common_msgs::JointControlMode::PARK;
                commandFsm->park();
            }
            break;
        case nasa_r2_common_msgs::JointControlMode::PARK:
        case nasa_r2_common_msgs::JointControlMode::NEUTRAL:
        case nasa_r2_common_msgs::JointControlMode::OFF:
        case nasa_r2_common_msgs::JointControlMode::INVALID:
        case nasa_r2_common_msgs::JointControlMode::IGNORE:
        default:
            break;
    }

    actualStatesLocal.clearFaultMode = commandStatesLocal.clearFaultMode;

    return actualStatesLocal;
}

nasa_r2_common_msgs::JointControlData JointControlManagerWrist::getCommandStates(void)
{
    return commandFsm->getStates();
}

void JointControlManagerWrist::setCommandStates(nasa_r2_common_msgs::JointControlData command)
{
    nasa_r2_common_msgs::JointControlData actualStatesLocal = getActualStates();
    switch(command.controlMode.state)
    {
        case nasa_r2_common_msgs::JointControlMode::BOOTLOADER:
            if (actualStatesLocal.controlMode.state != nasa_r2_common_msgs::JointControlMode::BOOTLOADER)
            {
                RCS::Logger::getCategory("gov.nasa.JointControlManagerWrist") << log4cpp::Priority::WARN << "BOOTLOADER command not allowed for " << mechanism << "; commanding park()";
                commandFsm->park();
            }
            break;
        case nasa_r2_common_msgs::JointControlMode::OFF:
            if(actualStatesLocal.controlMode.state == nasa_r2_common_msgs::JointControlMode::BOOTLOADER)
            {
                //! automatically calibrate from bootloader to off
                command.calibrationMode.state = nasa_r2_common_msgs::JointControlCalibrationMode::ENABLE;
            }
            commandFsm->off();
            break;
        case nasa_r2_common_msgs::JointControlMode::PARK:
            if(actualStatesLocal.controlMode.state == nasa_r2_common_msgs::JointControlMode::BOOTLOADER)
            {
                //! automatically calibrate from bootloader to park
                command.calibrationMode.state = nasa_r2_common_msgs::JointControlCalibrationMode::ENABLE;
            }
            commandFsm->park();
            break;
        case nasa_r2_common_msgs::JointControlMode::NEUTRAL:
            if(actualStatesLocal.controlMode.state == nasa_r2_common_msgs::JointControlMode::BOOTLOADER)
            {
                //! automatically calibrate from bootloader to neutral
                command.calibrationMode.state = nasa_r2_common_msgs::JointControlCalibrationMode::ENABLE;
            }
            commandFsm->neutral();
            break;
        case nasa_r2_common_msgs::JointControlMode::DRIVE:
            if(actualStatesLocal.calibrationMode.state == nasa_r2_common_msgs::JointControlCalibrationMode::DISABLE)
            {
                commandFsm->drive();
            }
            else
            {
                RCS::Logger::getCategory("gov.nasa.JointControlManagerWrist") << log4cpp::Priority::WARN << "Command transition to JointControlMode::DRIVE on mechanism: " << mechanism << " can not occur until it has been calibrated";
            }
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
            RCS::Logger::getCategory("gov.nasa.JointControlManagerWrist") << log4cpp::Priority::WARN << "JointControlCalibrationMode::DISABLE not allowed for " << mechanism << "; commanding <nothing>";
            break;
        case nasa_r2_common_msgs::JointControlCalibrationMode::ENABLE:
            commandFsm->enableCalibrationMode();
            commandFsm->disableCalibrationMode();
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

bool JointControlManagerWrist::verifyStates(void)
{
    prevActualStates = actualStates;
    actualStates     = getActualStates();
    commandStates    = getCommandStates();

    return verifyControlModeState() && verifyCommandModeState() && verifyCalibrationModeState() && verifyClearFaultModeState();
}

bool JointControlManagerWrist::verifyControlModeState(void)
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
            RCS::Logger::getCategory("gov.nasa.JointControlManagerWrist") << log4cpp::Priority::INFO << "Joint faulted: " << mechanism;
        }
        return false;
    }

    // Handle other state mismatches
    ros::Duration timeSinceBadStart = ros::Time::now() - controlModeBadStart;
    if(timeSinceBadStart.toSec() > timeLimit)
    {
        RCS::Logger::getCategory("gov.nasa.JointControlManagerWrist") << log4cpp::Priority::INFO << "Timed out waiting for " << jointControlModeToString(commandStates.controlMode) << " received " << jointControlModeToString(actualStates.controlMode) << " on mechanism: " << mechanism;
        controlModeBadStart = ros::Time::now();
        return false;
    }
    return true;
}

bool JointControlManagerWrist::verifyCommandModeState(void)
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
        RCS::Logger::getCategory("gov.nasa.JointControlManagerWrist") << log4cpp::Priority::INFO << "Timed out waiting for " << jointControlCommandModeToString(commandStates.commandMode) << " received " << jointControlCommandModeToString(actualStates.commandMode) << " on mechanism: " << mechanism;
        commandModeBadStart = ros::Time::now();
        return false;
    }
    return true;
}

bool JointControlManagerWrist::verifyCalibrationModeState(void)
{
    if(actualStates.calibrationMode.state == commandStates.calibrationMode.state)
    {
        calibrationModeBadStart = ros::Time::now();
        return true;
    }

    // Handle state mismatches
    ros::Duration timeSinceBadStart = ros::Time::now() - calibrationModeBadStart;
    if(timeSinceBadStart.toSec() > timeLimit)
    {
        RCS::Logger::getCategory("gov.nasa.JointControlManagerWrist") << log4cpp::Priority::INFO << "Timed out waiting for " << jointControlCalibrationModeToString(commandStates.calibrationMode) << " received " << jointControlCalibrationModeToString(actualStates.calibrationMode) << " on mechanism: " << mechanism;
        calibrationModeBadStart = ros::Time::now();
        return false;
    }
    return true;
}

bool JointControlManagerWrist::verifyClearFaultModeState(void)
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
        RCS::Logger::getCategory("gov.nasa.JointControlManagerWrist") << log4cpp::Priority::INFO << "Timed out waiting for " << jointControlClearFaultModeToString(commandStates.clearFaultMode) << " received " << jointControlClearFaultModeToString(actualStates.clearFaultMode) << " on mechanism: " << mechanism;
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
std::string JointControlManagerWrist::buildFaultString()
{
    verifyStates();

    std::string outStr("");
    if(actualStates.controlMode.state == nasa_r2_common_msgs::JointControlMode::FAULTED)
    {
        outStr.append("FAULT");

        if(io.getLiveCoeff(PitchLimitLiveCoeffName)            == 1) { outStr.append(", PitchLimit"); }
        if(io.getLiveCoeff(YawLimitLiveCoeffName)              == 1) { outStr.append(", YawLimit"); }
        if(io.getLiveCoeff(LittlesideSliderLimitLiveCoeffName) == 1) { outStr.append(", LittlesideSliderLimit"); }
        if(io.getLiveCoeff(ThumbsideSliderLimitLiveCoeffName)  == 1) { outStr.append(", ThumbsideSliderLimit"); }
        if(io.getLiveCoeff(SensorErrorLiveCoeffName)           == 1) { outStr.append(", SensorError"); }
        if(io.getLiveCoeff(SliderDiffErrorLiveCoeffName)       == 1) { outStr.append(", SliderDiffError"); }
    }
    else
    {
        outStr.append("none");
    }

    return outStr;
}
