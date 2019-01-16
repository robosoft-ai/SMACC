#include "nasa_robodyn_mechanisms_core/JointCommandGripper.h"

using namespace std;
using namespace log4cpp;
namespace SU = StringUtilities;

JointCommandGripper::JointCommandGripper(const std::string& mechanism, IoFunctions ioFunctions)
    : JointCommandInterface(mechanism, ioFunctions),
      completeMessageSize(3)
{
    if (mechanism == "")
    {
        stringstream err;
        err << "Constructor requires mechanism be non-empty.";
        RCS::Logger::log("gov.nasa.robonet.JointCommandGripper", Priority::FATAL, err.str());
        throw invalid_argument(err.str());
    }

    if (io.getFloat.empty() or 
        io.setFloat.empty() or 
        io.getMotorCoeff.empty() or
        io.getCommandFile.empty())
    {
        stringstream err;
        err << "Constructor requires 'io.getFloat', 'io.setFloat', 'io.getMotorCoeff', and 'io.getCommandFile' be non-empty.";
        RCS::Logger::log("gov.nasa.robonet.JointCommandGripper", Priority::FATAL, err.str());
        throw invalid_argument(err.str());
    }

    roboDynJoints.push_back(mechanism);
    jawLeftName  = SU::getRoboDynEverythingButJoint(mechanism) + SU::TOKEN_DELIMITER + string("jawLeft");
    jawRightName = SU::getRoboDynEverythingButJoint(mechanism) + SU::TOKEN_DELIMITER + string("jawRight");

    loadCoeffs();
}

JointCommandGripper::~JointCommandGripper()
{}

void JointCommandGripper::loadCoeffs()
{
    std::string parameterFile = io.getCommandFile(mechanism);

    //! Initialize messages
    simpleMeasuredStateMsg.name.resize(completeMessageSize);
    simpleMeasuredStateMsg.position.resize(completeMessageSize);
    simpleMeasuredStateMsg.velocity.resize(completeMessageSize);
    simpleMeasuredStateMsg.effort.resize(completeMessageSize);

    simpleMeasuredStateMsg.name[0] = roboDynJoints[0];
    simpleMeasuredStateMsg.name[1] = jawLeftName;
    simpleMeasuredStateMsg.name[2] = jawRightName;

    completeMeasuredStateMsg.name.resize(completeMessageSize);
    completeMeasuredStateMsg.position.resize(completeMessageSize);
    completeMeasuredStateMsg.velocity.resize(completeMessageSize);
    completeMeasuredStateMsg.effort.resize(completeMessageSize);

    completeMeasuredStateMsg.name[0] = roboDynJoints[0];
    completeMeasuredStateMsg.name[1] = jawLeftName;
    completeMeasuredStateMsg.name[2] = jawRightName;

    // just send an empty commanded message to bypass robodyn
//    commandedStateMsg.name.resize(1);
//    commandedStateMsg.desiredPosition.resize(1);
//    commandedStateMsg.desiredPositionVelocityLimit.resize(1);
//    commandedStateMsg.feedForwardTorque.resize(1);
//    commandedStateMsg.proportionalGain.resize(1);
//    commandedStateMsg.derivativeGain.resize(1);
//    commandedStateMsg.integralGain.resize(1);
//    commandedStateMsg.positionLoopTorqueLimit.resize(1);
//    commandedStateMsg.positionLoopWindupLimit.resize(1);
//    commandedStateMsg.torqueLoopVelocityLimit.resize(1);

//    commandedStateMsg.name[0] = roboDynJoints[0];

    jointCapabilityMsg.name.resize(1);
    jointCapabilityMsg.positionLimitMax.resize(1);
    jointCapabilityMsg.positionLimitMin.resize(1);
    jointCapabilityMsg.torqueLimit.resize(1);

    jointCapabilityMsg.name[0] = roboDynJoints[0];

    positionVals.resize(completeMessageSize, 0.);
    velocityVals.resize(completeMessageSize, 0.);
    effortVals.resize(completeMessageSize, 0.);

    //! Parse parameter file
    TiXmlDocument file(parameterFile.c_str());
    bool loadOkay = file.LoadFile();
    if (!loadOkay)
    {
        stringstream err;
        err << "Failed to load file [" << parameterFile << "]";
        RCS::Logger::log("gov.nasa.robonet.JointCommandGripper", Priority::FATAL, err.str());
        throw runtime_error(err.str());
    }
    TiXmlHandle doc(&file);
    RCS::Logger::log("gov.nasa.robonet.JointCommandGripper", Priority::INFO, "CommandFile [" + parameterFile + "] successfully loaded.");

    // Print the XML file's contents
    //cout << "Successfully loaded file: " << parameterFile << endl;
    //TiXmlPrinter printer;
    //printer.SetIndent("\t");
    //file.Accept(&printer);
    //cout << printer.Str() << endl;

    // Check for ApiMap
    TiXmlHandle parametersElement(doc.FirstChildElement("ApiMap"));

    //! @todo remove the hardcoded "7"s below... should somehow be provided
    if (parametersElement.ToElement())
    {
        //! Status Elements
        jointPositionStatusElement =                             StringUtilities::makeFullyQualifiedRoboDynElement( roboDynJoints[0], ApiMap::getXmlElementValue(parametersElement, "JointPositionStatus"));
        jawLeftPositionStatusElement =                           StringUtilities::makeFullyQualifiedRoboDynElement( roboDynJoints[0], ApiMap::getXmlElementValue(parametersElement, "JawLeftPositionStatus"));
        jawRightPositionStatusElement =                          StringUtilities::makeFullyQualifiedRoboDynElement( roboDynJoints[0], ApiMap::getXmlElementValue(parametersElement, "JawRightPositionStatus"));
        jointVelocityStatusElement =                             StringUtilities::makeFullyQualifiedRoboDynElement( roboDynJoints[0], ApiMap::getXmlElementValue(parametersElement, "JointVelocityStatus"));
        // jawLeftVelocityStatusElement =                           StringUtilities::makeFullyQualifiedRoboDynElement( roboDynJoints[0], ApiMap::getXmlElementValue(parametersElement, "JawLeftVelocityStatus"));
        // jawRightVelocityStatusElement =                          StringUtilities::makeFullyQualifiedRoboDynElement( roboDynJoints[0], ApiMap::getXmlElementValue(parametersElement, "JawRightVelocityStatus"));
        jawLeftTorqueStatusElement =                             StringUtilities::makeFullyQualifiedRoboDynElement( roboDynJoints[0], ApiMap::getXmlElementValue(parametersElement, "JawLeftTorqueStatus"));
        jawRightTorqueStatusElement =                            StringUtilities::makeFullyQualifiedRoboDynElement( roboDynJoints[0], ApiMap::getXmlElementValue(parametersElement, "JawRightTorqueStatus"));
        //! Command Elements
        desiredPositionCommandElement =                          StringUtilities::makeFullyQualifiedRoboDynElement( roboDynJoints[0], ApiMap::getXmlElementValue(parametersElement, "DesiredPositionCommand"));
        desiredPositionVelocityLimitCommandElement =             StringUtilities::makeFullyQualifiedRoboDynElement( roboDynJoints[0], ApiMap::getXmlElementValue(parametersElement, "DesiredPositionVelocityLimitCommand"));
        feedForwardTorqueCommandElement =                        StringUtilities::makeFullyQualifiedRoboDynElement( roboDynJoints[0], ApiMap::getXmlElementValue(parametersElement, "FeedForwardTorqueCommand"));
        proportionalGainCommandElement =                         StringUtilities::makeFullyQualifiedRoboDynElement( roboDynJoints[0], ApiMap::getXmlElementValue(parametersElement, "ProportionalGainCommand"));
        derivativeGainCommandElement =                           StringUtilities::makeFullyQualifiedRoboDynElement( roboDynJoints[0], ApiMap::getXmlElementValue(parametersElement, "DerivativeGainCommand"));
        integralGainCommandElement =                             StringUtilities::makeFullyQualifiedRoboDynElement( roboDynJoints[0], ApiMap::getXmlElementValue(parametersElement, "IntegralGainCommand"));
        positionLoopTorqueLimitCommandElement =                  StringUtilities::makeFullyQualifiedRoboDynElement( roboDynJoints[0], ApiMap::getXmlElementValue(parametersElement, "PositionLoopTorqueLimitCommand"));
        positionLoopWindupLimitCommandElement =                  StringUtilities::makeFullyQualifiedRoboDynElement( roboDynJoints[0], ApiMap::getXmlElementValue(parametersElement, "PositionLoopWindupLimitCommand"));
        torqueLoopVelocityLimitCommandElement =                  StringUtilities::makeFullyQualifiedRoboDynElement( roboDynJoints[0], ApiMap::getXmlElementValue(parametersElement, "TorqueLoopVelocityLimitCommand"));
        //! Capability Elements
        jointCapabilityMsg.positionLimitMax[0] =     io.getMotorCoeff(StringUtilities::makeFullyQualifiedRoboDynElement( roboDynJoints[0], ApiMap::getXmlElementValue(parametersElement, "PositionLimitMax")));
        jointCapabilityMsg.positionLimitMin[0] =     io.getMotorCoeff(StringUtilities::makeFullyQualifiedRoboDynElement( roboDynJoints[0], ApiMap::getXmlElementValue(parametersElement, "PositionLimitMin")));
        jointCapabilityMsg.torqueLimit[0] =          0.;
    }
    else
    {
        stringstream err;
        err << "The file " << parameterFile << " has no element named [ApiMap]";
        RCS::Logger::log("gov.nasa.robonet.JointCommandGripper", Priority::ERROR, err.str());
        throw runtime_error(err.str());
    }
}

void JointCommandGripper::updateMeasuredState(nasa_r2_common_msgs::JointControlData msg)
{
    // If coeffs are not loaded, override with zeros RDEV-1075
    if (msg.coeffState.state == nasa_r2_common_msgs::JointControlCoeffState::LOADED)
    {
        //! joint
        positionVals[0] = io.getFloat(jointPositionStatusElement);
        velocityVals[0] = io.getFloat(jointVelocityStatusElement);
        effortVals[0]   = 0;

        //! left jaw
        positionVals[1] = io.getFloat(jawLeftPositionStatusElement);
        // velocityVals[1] = io.getFloat(jawLeftVelocityStatusElement);
        velocityVals[1] = 0;
        effortVals[1]   = io.getFloat(jawLeftTorqueStatusElement);

        //! right jaw
        positionVals[2] = io.getFloat(jawRightPositionStatusElement);
        // velocityVals[2] = io.getFloat(jawRightVelocityStatusElement);
        velocityVals[2] = 0;
        effortVals[2]   = io.getFloat(jawRightTorqueStatusElement);
    }
    else
    {
        positionVals.assign(completeMessageSize, 0.0);
        velocityVals.assign(completeMessageSize, 0.0);
        effortVals.assign(completeMessageSize, 0.0);
    }
}

sensor_msgs::JointState JointCommandGripper::getSimpleMeasuredState()
{
    simpleMeasuredStateMsg.header.stamp = ros::Time::now();

    simpleMeasuredStateMsg.position = positionVals;
    simpleMeasuredStateMsg.velocity = velocityVals;
    simpleMeasuredStateMsg.effort   = effortVals;

    return simpleMeasuredStateMsg;
}

sensor_msgs::JointState JointCommandGripper::getCompleteMeasuredState()
{
    completeMeasuredStateMsg.header.stamp = ros::Time::now();

    completeMeasuredStateMsg.position = positionVals;
    completeMeasuredStateMsg.velocity = velocityVals;
    completeMeasuredStateMsg.effort   = effortVals;

    return completeMeasuredStateMsg;
}

void JointCommandGripper::setFaultState()
{
}

nasa_r2_common_msgs::JointCommand JointCommandGripper::getCommandedState()
{
    // just send an empty message to bypass robodyn
//    commandedStateMsg.header.stamp = ros::Time::now();

//    commandedStateMsg.desiredPosition[0]              = io.getFloat(desiredPositionCommandElement);
//    commandedStateMsg.desiredPositionVelocityLimit[0] = io.getFloat(desiredPositionVelocityLimitCommandElement);
//    commandedStateMsg.feedForwardTorque[0]            = io.getFloat(feedForwardTorqueCommandElement);
//    commandedStateMsg.proportionalGain[0]             = io.getFloat(proportionalGainCommandElement);
//    commandedStateMsg.derivativeGain[0]               = io.getFloat(derivativeGainCommandElement);
//    commandedStateMsg.integralGain[0]                 = io.getFloat(integralGainCommandElement);
//    commandedStateMsg.positionLoopTorqueLimit[0]      = io.getFloat(positionLoopTorqueLimitCommandElement);
//    commandedStateMsg.positionLoopWindupLimit[0]      = io.getFloat(positionLoopWindupLimitCommandElement);
    // commandedStateMsg.torqueLoopVelocityLimit[0]      = io.getFloat(torqueLoopVelocityLimitCommandElement);

    return commandedStateMsg;
}

void JointCommandGripper::setCommand(nasa_r2_common_msgs::JointCommand msg, nasa_r2_common_msgs::JointControlData)
{
    if (msg.name.size() != 1)
    {
        stringstream err;
        err << "setCommand() requires one and only one entry in JointCommand.name when setCommand() is called.";
        RCS::Logger::log("gov.nasa.robonet.JointCommandGripper", Priority::ERROR, err.str());
        throw runtime_error(err.str());
    }

    if ((msg.desiredPosition.size() > 1) ||
        (msg.desiredPositionVelocityLimit.size() > 1) || 
        (msg.feedForwardTorque.size() > 1) || 
        (msg.proportionalGain.size() > 1) || 
        (msg.derivativeGain.size() > 1) || 
        (msg.integralGain.size() > 1) || 
        (msg.positionLoopTorqueLimit.size() > 1) || 
        (msg.positionLoopWindupLimit.size() > 1) || 
        (msg.torqueLoopVelocityLimit.size() > 1))
    {
        stringstream err;
        err << "setCommand() requires at most one entry in JointCommand.* when setCommand() is called.";
        RCS::Logger::log("gov.nasa.robonet.JointCommandGripper", Priority::ERROR, err.str());
        throw runtime_error(err.str());
    }

    if (msg.name[0] != roboDynJoints[0])
    {
        stringstream err;
        err << "setCommand() received JointCommand message for joint [" << msg.name[0] << "]. It expected [" << roboDynJoints[0] << "].";
        RCS::Logger::log("gov.nasa.robonet.JointCommandGripper", Priority::ERROR, err.str());
        throw runtime_error(err.str());
    }

    if (not msg.desiredPosition.empty())
    {
        io.setFloat(desiredPositionCommandElement, msg.desiredPosition[0]);
    }
    if (not msg.desiredPositionVelocityLimit.empty())
    {
        io.setFloat(desiredPositionVelocityLimitCommandElement, msg.desiredPositionVelocityLimit[0]);
    }
    if (not msg.feedForwardTorque.empty())
    {
        io.setFloat(feedForwardTorqueCommandElement, msg.feedForwardTorque[0]);
    }
    if (not msg.proportionalGain.empty())
    {
        io.setFloat(proportionalGainCommandElement, msg.proportionalGain[0]);
    }
    if (not msg.derivativeGain.empty())
    {
        io.setFloat(derivativeGainCommandElement, msg.derivativeGain[0]);
    }
    if (not msg.integralGain.empty())
    {
        io.setFloat(integralGainCommandElement, msg.integralGain[0]);
    }
    if (not msg.positionLoopTorqueLimit.empty())
    {
        io.setFloat(positionLoopTorqueLimitCommandElement, msg.positionLoopTorqueLimit[0]);
    }
    if (not msg.positionLoopWindupLimit.empty())
    {
        io.setFloat(positionLoopWindupLimitCommandElement, msg.positionLoopWindupLimit[0]);
    }
    // if (msg.torqueLoopVelocityLimit.size())
    // {
    // //! @todo decide whether this should be included or not
    // io.setFloat(torqueLoopVelocityLimitCommandElement,      msg.torqueLoopVelocityLimit[0]);
    // }
}

nasa_r2_common_msgs::JointCapability JointCommandGripper::getCapability()
{
    return jointCapabilityMsg;
}
