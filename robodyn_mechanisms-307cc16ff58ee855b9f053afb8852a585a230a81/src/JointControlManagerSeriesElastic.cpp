/**
 * @file JointControlManagerSeriesElastic.cpp
 */

#include "nasa_robodyn_mechanisms_core/JointControlManagerSeriesElastic.h"

JointControlManagerSeriesElastic::JointControlManagerSeriesElastic(const std::string& mechanism, IoFunctions ioFunctions, double timeLimit, const std::string& type, NodeRegisterManagerPtr nodeRegisterManager)
    : JointControlManagerInterface(mechanism, ioFunctions, timeLimit, type), nodeRegisterManager(nodeRegisterManager)
{
    setParameters();

    // Initialize variables
    controlModeBadStart     = ros::Time::now();
    commandModeBadStart     = ros::Time::now();
    calibrationModeBadStart = ros::Time::now();

    // Instantiate state machines
    actualFsm  = boost::make_shared<JointControlActualFsmSeriesElastic> (mechanism, ioFunctions, nodeRegisterManager);
    commandFsm = boost::make_shared<JointControlCommandFsmSeriesElastic>(mechanism, ioFunctions, nodeRegisterManager);

    // Set constant control register bits for this type of joint
    nodeRegisterManager->setControlValue(mechanism, InitAPS2toAPS1ControlName, 0); // We think this is used for neck
}

JointControlManagerSeriesElastic::~JointControlManagerSeriesElastic()
{
    // Nothing
}

void JointControlManagerSeriesElastic::setParameters()
{
    std::string parameterFile = io.getControlFile(mechanism);

    //! Parse parameter file
    TiXmlDocument file(parameterFile.c_str());
    bool loadOkay = file.LoadFile();
    if (!loadOkay)
    {
        std::stringstream err;
        err << "Failed to load file [" << parameterFile << "]";
        RCS::Logger::log("gov.nasa.JointControlManagerSeriesElastic", log4cpp::Priority::FATAL, err.str());
        throw std::runtime_error(err.str());
    }
    TiXmlHandle doc(&file);
    RCS::Logger::log("gov.nasa.JointControlManagerSeriesElastic", log4cpp::Priority::INFO, "File [" + parameterFile + "] successfully loaded.");

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
        BridgeFaultStatusName     = ApiMap::getXmlElementValue(parametersElement, "BridgeFaultStatus");
        JointFaultStatusName      = ApiMap::getXmlElementValue(parametersElement, "JointFaultStatus");
        BusVoltageFaultStatusName = ApiMap::getXmlElementValue(parametersElement, "BusVoltageFaultStatus");
        ApsFaultStatusName        = ApiMap::getXmlElementValue(parametersElement, "ApsFaultStatus");
        Aps1TolFaultStatusName    = ApiMap::getXmlElementValue(parametersElement, "Aps1TolFaultStatus");
        Aps2TolFaultStatusName    = ApiMap::getXmlElementValue(parametersElement, "Aps2TolFaultStatus");
        EncDriftFaultStatusName   = ApiMap::getXmlElementValue(parametersElement, "EncDriftFaultStatus");
        VelocityFaultStatusName   = ApiMap::getXmlElementValue(parametersElement, "VelocityFaultStatus");
        LimitFaultStatusName      = ApiMap::getXmlElementValue(parametersElement, "LimitFaultStatus");
        CoeffsLoadedStatusName    = ApiMap::getXmlElementValue(parametersElement, "CoeffsLoadedStatus");
        CurrentFaultStatusName    = ApiMap::getXmlElementValue(parametersElement, "CurrentFaultStatus");

        // Control bit names
        InitAPS2toAPS1ControlName  = ApiMap::getXmlElementValue(parametersElement, "InitAPS2toAPS1Control");
    }
    else
    {
        std::stringstream err;
        err << "The file " << parameterFile << " has no element named [ApiMap]";
        RCS::Logger::log("gov.nasa.JointControlManagerSeriesElastic", log4cpp::Priority::ERROR, err.str());
        throw std::runtime_error(err.str());
    }
}

nasa_r2_common_msgs::JointControlData JointControlManagerSeriesElastic::getActualStates(void)
{
    return actualFsm->getStates();
}

nasa_r2_common_msgs::JointControlData JointControlManagerSeriesElastic::getCommandStates(void)
{
    return commandFsm->getStates();
}

void JointControlManagerSeriesElastic::setCommandStates(nasa_r2_common_msgs::JointControlData command)
{
    nasa_r2_common_msgs::JointControlData currentActualStates = getActualStates();

    switch(command.controlMode.state)
    {
        case nasa_r2_common_msgs::JointControlMode::BOOTLOADER:
            //! Allowing OFF -> BOOTLOADER for #RDEV-1239
            if( (currentActualStates.controlMode.state == nasa_r2_common_msgs::JointControlMode::BOOTLOADER) || 
                (currentActualStates.controlMode.state == nasa_r2_common_msgs::JointControlMode::OFF)        )
            {
                commandFsm->bootLoader();
            }
            else
            {
                RCS::Logger::getCategory("gov.nasa.JointControlManagerSeriesElastic") << log4cpp::Priority::WARN << "Command transition to JointControlMode::" << jointControlModeToString(command.controlMode) << " not allowed on joint: " << mechanism << ", current actual state: " << jointControlModeToString(currentActualStates.controlMode);
            }
            break;
        case nasa_r2_common_msgs::JointControlMode::OFF:
            if( (currentActualStates.controlMode.state == nasa_r2_common_msgs::JointControlMode::BOOTLOADER) ||
                (currentActualStates.controlMode.state == nasa_r2_common_msgs::JointControlMode::OFF)        )
            {
                commandFsm->off();
            }
            else
            {
                RCS::Logger::getCategory("gov.nasa.JointControlManagerSeriesElastic") << log4cpp::Priority::WARN << "Command transition to JointControlMode::" << jointControlModeToString(command.controlMode) << " not allowed on joint: " << mechanism << ", current actual state: " << jointControlModeToString(currentActualStates.controlMode);
            }
            break;
        case nasa_r2_common_msgs::JointControlMode::PARK:
            if( (currentActualStates.controlMode.state == nasa_r2_common_msgs::JointControlMode::FAULTED) ||
                (currentActualStates.controlMode.state == nasa_r2_common_msgs::JointControlMode::OFF)     ||
                (currentActualStates.controlMode.state == nasa_r2_common_msgs::JointControlMode::PARK)    ||
                (currentActualStates.controlMode.state == nasa_r2_common_msgs::JointControlMode::NEUTRAL) ||
                (currentActualStates.controlMode.state == nasa_r2_common_msgs::JointControlMode::DRIVE)   )
            {
                commandFsm->park();
            }
            else
            {
                RCS::Logger::getCategory("gov.nasa.JointControlManagerSeriesElastic") << log4cpp::Priority::WARN << "Command transition to JointControlMode::" << jointControlModeToString(command.controlMode) << " not allowed on joint: " << mechanism << ", current actual state: " << jointControlModeToString(currentActualStates.controlMode);
            }
            break;
        case nasa_r2_common_msgs::JointControlMode::NEUTRAL:
            if( (currentActualStates.controlMode.state == nasa_r2_common_msgs::JointControlMode::FAULTED) ||
                (currentActualStates.controlMode.state == nasa_r2_common_msgs::JointControlMode::OFF)     ||
                (currentActualStates.controlMode.state == nasa_r2_common_msgs::JointControlMode::PARK)    ||
                (currentActualStates.controlMode.state == nasa_r2_common_msgs::JointControlMode::NEUTRAL) )
            {
                commandFsm->neutral();
            }
            else
            {
                RCS::Logger::getCategory("gov.nasa.JointControlManagerSeriesElastic") << log4cpp::Priority::WARN << "Command transition to JointControlMode::" << jointControlModeToString(command.controlMode) << " not allowed on joint: " << mechanism << ", current actual state: " << jointControlModeToString(currentActualStates.controlMode);
            }
            break;
        case nasa_r2_common_msgs::JointControlMode::DRIVE:
            if( (currentActualStates.controlMode.state == nasa_r2_common_msgs::JointControlMode::PARK)    ||
                (currentActualStates.controlMode.state == nasa_r2_common_msgs::JointControlMode::NEUTRAL) ||
                (currentActualStates.controlMode.state == nasa_r2_common_msgs::JointControlMode::DRIVE)   )
            {
                commandFsm->drive();
            }
            else
            {
                RCS::Logger::getCategory("gov.nasa.JointControlManagerSeriesElastic") << log4cpp::Priority::WARN << "Command transition to JointControlMode::" << jointControlModeToString(command.controlMode) << " not allowed on joint: " << mechanism << ", current actual state: " << jointControlModeToString(currentActualStates.controlMode);
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

bool JointControlManagerSeriesElastic::verifyStates(void)
{
    prevActualStates = actualStates;
    actualStates     = getActualStates();
    commandStates    = getCommandStates();

    return verifyControlModeState() && verifyCommandModeState() && verifyCalibrationModeState() && verifyClearFaultModeState();
}

bool JointControlManagerSeriesElastic::verifyControlModeState(void)
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
            RCS::Logger::getCategory("gov.nasa.JointControlManagerSeriesElastic") << log4cpp::Priority::INFO << "Joint faulted: " << mechanism;
        }
        return false;
    }

    // Handle other state mismatches
    ros::Duration timeSinceBadStart = ros::Time::now() - controlModeBadStart;
    if(timeSinceBadStart.toSec() > timeLimit)
    {
        RCS::Logger::getCategory("gov.nasa.JointControlManagerSeriesElastic") << log4cpp::Priority::INFO << "Timed out waiting for " << jointControlModeToString(commandStates.controlMode) << " received " << jointControlModeToString(actualStates.controlMode) << " on mechanism: " << mechanism;
        controlModeBadStart = ros::Time::now();
        return false;
    }
    return true;
}

bool JointControlManagerSeriesElastic::verifyCommandModeState(void)
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
        RCS::Logger::getCategory("gov.nasa.JointControlManagerSeriesElastic") << log4cpp::Priority::INFO << "Timed out waiting for " << jointControlCommandModeToString(commandStates.commandMode) << " received " << jointControlCommandModeToString(actualStates.commandMode) << " on mechanism: " << mechanism;
        commandModeBadStart = ros::Time::now();
        return false;
    }
    return true;
}

bool JointControlManagerSeriesElastic::verifyCalibrationModeState(void)
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
        RCS::Logger::getCategory("gov.nasa.JointControlManagerSeriesElastic") << log4cpp::Priority::INFO << "Timed out waiting for " << jointControlCalibrationModeToString(commandStates.calibrationMode) << " received " << jointControlCalibrationModeToString(actualStates.calibrationMode) << " on mechanism: " << mechanism;
        calibrationModeBadStart = ros::Time::now();
        return false;
    }
    return true;
}

bool JointControlManagerSeriesElastic::verifyClearFaultModeState(void)
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
        RCS::Logger::getCategory("gov.nasa.JointControlManagerSeriesElastic") << log4cpp::Priority::INFO << "Timed out waiting for " << jointControlClearFaultModeToString(commandStates.clearFaultMode) << " received " << jointControlClearFaultModeToString(actualStates.clearFaultMode) << " on mechanism: " << mechanism;
        clearFaultModeBadStart = ros::Time::now();
        return false;
    }
    return true;
}

std::string JointControlManagerSeriesElastic::buildFaultString()
{
    verifyStates();

    std::string outStr("");
    if(actualStates.controlMode.state == nasa_r2_common_msgs::JointControlMode::FAULTED)
    {
        outStr.append("FAULT");

        if(nodeRegisterManager->getStatusValue(mechanism, ProcAliveStatusName)       == 0) { outStr.append(", ProcNotAlive"); }
        if(nodeRegisterManager->getStatusValue(mechanism, CommAliveStatusName)       == 0) { outStr.append(", CommNotAlive"); }
        if(nodeRegisterManager->getStatusValue(mechanism, BridgeFaultStatusName)     == 1) { outStr.append(", BridgeFault"); }
        if(nodeRegisterManager->getStatusValue(mechanism, JointFaultStatusName)      == 1) { outStr.append(", JointFault"); }
        if(nodeRegisterManager->getStatusValue(mechanism, BusVoltageFaultStatusName) == 1) { outStr.append(", BusVoltageFault"); }
        if(nodeRegisterManager->getStatusValue(mechanism, ApsFaultStatusName)        == 1) { outStr.append(", ApsFault"); }
        if(nodeRegisterManager->getStatusValue(mechanism, Aps1TolFaultStatusName)    == 1) { outStr.append(", Aps1TolFault"); }
        if(nodeRegisterManager->getStatusValue(mechanism, Aps2TolFaultStatusName)    == 1) { outStr.append(", Aps2TolFault"); }
        if(nodeRegisterManager->getStatusValue(mechanism, EncDriftFaultStatusName)   == 1) { outStr.append(", EncDriftFault"); }
        if(nodeRegisterManager->getStatusValue(mechanism, VelocityFaultStatusName)   == 1) { outStr.append(", VelocityFault"); }
        if(nodeRegisterManager->getStatusValue(mechanism, LimitFaultStatusName)      == 1) { outStr.append(", LimitFault"); }
        if(nodeRegisterManager->getStatusValue(mechanism, CoeffsLoadedStatusName)    == 0) { outStr.append(", CoeffsNotLoaded"); }
        if(nodeRegisterManager->getStatusValue(mechanism, CurrentFaultStatusName)    == 1) { outStr.append(", CurrentFault"); }
    }
    else
    {
        outStr.append("none");
    }

    return outStr;
}
