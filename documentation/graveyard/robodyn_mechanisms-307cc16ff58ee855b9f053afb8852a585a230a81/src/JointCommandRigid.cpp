#include "nasa_robodyn_mechanisms_core/JointCommandRigid.h"

using namespace std;
using namespace log4cpp;
namespace SU = StringUtilities;

JointCommandRigid::JointCommandRigid(const std::string& mechanism, IoFunctions ioFunctions)
    : JointCommandInterface(mechanism, ioFunctions),
      completeMessageSize(2)
{
    if (mechanism == "")
    {
        stringstream err;
        err << "Constructor requires mechanism be non-empty.";
        RCS::Logger::log("gov.nasa.robonet.JointCommandRigid", Priority::FATAL, err.str());
        throw invalid_argument(err.str());
    }

    if (io.getFloat.empty()  or
        io.setFloat.empty()  or
        io.getUInt32.empty() or
        io.getMotorCoeff.empty()  or
        io.hasBrainstemCoeff.empty()  or
        io.getBrainstemCoeff.empty()  or
        io.getCommandFile.empty())
    {
        stringstream err;
        err << "Constructor requires 'io.getFloat', 'io.setFloat', 'io.getUInt32', 'io.getMotorCoeff', 'io.getBrainstemCoeff', and 'io.getCommandFile' be non-empty.";
        RCS::Logger::log("gov.nasa.robonet.JointCommandRigid", Priority::FATAL, err.str());
        throw invalid_argument(err.str());
    }

    roboDynJoints.push_back(mechanism);
    encoderName = SU::getRoboDynEverythingButJoint(mechanism) + SU::TOKEN_DELIMITER + string("encoder") + SU::getNumber(SU::getRoboDynJoint(mechanism));

    loadCoeffs();
}

JointCommandRigid::~JointCommandRigid()
{}

void JointCommandRigid::loadCoeffs()
{
    std::string parameterFile = io.getCommandFile(mechanism);

    //! Initialize messages
    simpleMeasuredStateMsg.name.resize(1);
    simpleMeasuredStateMsg.position.resize(1);
    simpleMeasuredStateMsg.velocity.resize(1);
    simpleMeasuredStateMsg.effort.resize(1);

    simpleMeasuredStateMsg.name[0] = roboDynJoints[0];

    completeMeasuredStateMsg.name.resize(completeMessageSize);
    completeMeasuredStateMsg.position.resize(completeMessageSize);
    completeMeasuredStateMsg.velocity.resize(completeMessageSize);
    completeMeasuredStateMsg.effort.resize(completeMessageSize);

    completeMeasuredStateMsg.name[0] = roboDynJoints[0];
    completeMeasuredStateMsg.name[1] = encoderName;

    commandedStateMsg.name.resize(1);
    commandedStateMsg.desiredPosition.resize(1);
    commandedStateMsg.desiredPositionVelocityLimit.resize(1);
    commandedStateMsg.feedForwardTorque.resize(1);
    commandedStateMsg.proportionalGain.resize(1);
    commandedStateMsg.derivativeGain.resize(1);
    commandedStateMsg.integralGain.resize(1);
    commandedStateMsg.positionLoopTorqueLimit.resize(1);
    commandedStateMsg.positionLoopWindupLimit.resize(1);
    commandedStateMsg.torqueLoopVelocityLimit.resize(1);

    commandedStateMsg.name[0] = roboDynJoints[0];

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
        RCS::Logger::log("gov.nasa.robonet.JointCommandRigid", Priority::FATAL, err.str());
        throw runtime_error(err.str());
    }
    TiXmlHandle doc(&file);
    RCS::Logger::log("gov.nasa.robonet.JointCommandRigid", Priority::INFO, "CommandFile [" + parameterFile + "] successfully loaded.");

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
        //! Status Elements
        jointPositionStatusElement =                 StringUtilities::makeFullyQualifiedRoboDynElement( roboDynJoints[0], ApiMap::getXmlElementValue(parametersElement, "JointPositionStatus"));
        encoderPositionStatusElement =               StringUtilities::makeFullyQualifiedRoboDynElement( roboDynJoints[0], ApiMap::getXmlElementValue(parametersElement, "EncoderPositionStatus"));
        jointVelocityStatusElement =                 StringUtilities::makeFullyQualifiedRoboDynElement( roboDynJoints[0], ApiMap::getXmlElementValue(parametersElement, "JointVelocityStatus"));
        encoderVelocityStatusElement =               StringUtilities::makeFullyQualifiedRoboDynElement( roboDynJoints[0], ApiMap::getXmlElementValue(parametersElement, "EncoderVelocityStatus"));
        //! Command Elements
        desiredPositionCommandElement =              StringUtilities::makeFullyQualifiedRoboDynElement( roboDynJoints[0], ApiMap::getXmlElementValue(parametersElement, "DesiredPositionCommand"));
        desiredPositionVelocityLimitCommandElement = StringUtilities::makeFullyQualifiedRoboDynElement( roboDynJoints[0], ApiMap::getXmlElementValue(parametersElement, "DesiredPositionVelocityLimitCommand"));
        //! Capability Elements
        jointKinematicOffset =                       io.getBrainstemCoeff(StringUtilities::makeFullyQualifiedRoboDynElement( roboDynJoints[0], ApiMap::getXmlElementValue(parametersElement, "APSKinematicOffset")));
        jointCapabilityMsg.positionLimitMax[0] =     io.getMotorCoeff(StringUtilities::makeFullyQualifiedRoboDynElement( roboDynJoints[0], ApiMap::getXmlElementValue(parametersElement, "PositionLimitMax")))
                + jointKinematicOffset;
        jointCapabilityMsg.positionLimitMin[0] =     io.getMotorCoeff(StringUtilities::makeFullyQualifiedRoboDynElement( roboDynJoints[0], ApiMap::getXmlElementValue(parametersElement, "PositionLimitMin")))
                + jointKinematicOffset;

        //! Set robot limits
        brakePwmCommandElement =                     StringUtilities::makeFullyQualifiedRoboDynElement( roboDynJoints[0], ApiMap::getXmlElementValue(parametersElement, "BrakePWM"));
        motComLimitCommandElement =                  StringUtilities::makeFullyQualifiedRoboDynElement( roboDynJoints[0], ApiMap::getXmlElementValue(parametersElement, "MotComLimit"));
        if(not(io.hasBrainstemCoeff(brakePwmCommandElement)))
        {
            stringstream err;
            err << "The BrainstemCoeff " << brakePwmCommandElement << " does not exist";
            RCS::Logger::log("gov.nasa.robonet.JointCommandSeriesElastic", Priority::ERROR, err.str());
            throw runtime_error(err.str());
        }
        if(not(io.hasBrainstemCoeff(motComLimitCommandElement)))
        {
            stringstream err;
            err << "The BrainstemCoeff " << motComLimitCommandElement << " does not exist";
            RCS::Logger::log("gov.nasa.robonet.JointCommandSeriesElastic", Priority::ERROR, err.str());
            throw runtime_error(err.str());
        }
        io.setUInt16(brakePwmCommandElement,    io.getBrainstemCoeff(brakePwmCommandElement));
        io.setUInt16(motComLimitCommandElement, io.getBrainstemCoeff(motComLimitCommandElement));
    }
    else
    {
        stringstream err;
        err << "The file " << parameterFile << " has no element named [ApiMap]";
        RCS::Logger::log("gov.nasa.robonet.JointCommandRigid", Priority::ERROR, err.str());
        throw runtime_error(err.str());
    }
}

void JointCommandRigid::updateMeasuredState(nasa_r2_common_msgs::JointControlData msg)
{
    // If coeffs are not loaded, override with zeros RDEV-1075
    if (msg.coeffState.state == nasa_r2_common_msgs::JointControlCoeffState::LOADED)
    {
        //! joint
        positionVals[0] = io.getFloat(jointPositionStatusElement)+jointKinematicOffset;
        velocityVals[0] = io.getFloat(jointVelocityStatusElement);
        effortVals[0]   = 0;

        //! encoder
        positionVals[1] = io.getFloat(encoderPositionStatusElement);
        velocityVals[1] = io.getFloat(encoderVelocityStatusElement);
        effortVals[1]   = 0;
    }
    else
    {
        positionVals.assign(completeMessageSize, 0.0);
        velocityVals.assign(completeMessageSize, 0.0);
        effortVals.assign(completeMessageSize, 0.0);
    }
}

sensor_msgs::JointState JointCommandRigid::getSimpleMeasuredState()
{
    simpleMeasuredStateMsg.header.stamp = ros::Time::now();

    simpleMeasuredStateMsg.position[0] = positionVals[0];
    simpleMeasuredStateMsg.velocity[0] = velocityVals[0];
    simpleMeasuredStateMsg.effort[0]   = effortVals[0];

    return simpleMeasuredStateMsg;
}

sensor_msgs::JointState JointCommandRigid::getCompleteMeasuredState()
{
    completeMeasuredStateMsg.header.stamp = ros::Time::now();

    completeMeasuredStateMsg.position = positionVals;
    completeMeasuredStateMsg.velocity = velocityVals;
    completeMeasuredStateMsg.effort   = effortVals;

    return completeMeasuredStateMsg;
}

void JointCommandRigid::setFaultState()
{
}

nasa_r2_common_msgs::JointCommand JointCommandRigid::getCommandedState()
{
    commandedStateMsg.header.stamp = ros::Time::now();

    commandedStateMsg.desiredPosition[0]              = io.getFloat(desiredPositionCommandElement)+jointKinematicOffset;
    commandedStateMsg.desiredPositionVelocityLimit[0] = io.getFloat(desiredPositionVelocityLimitCommandElement);

    return commandedStateMsg;
}

void JointCommandRigid::setCommand(nasa_r2_common_msgs::JointCommand msg, nasa_r2_common_msgs::JointControlData)
{
    if (msg.name.size() != 1)
    {
        stringstream err;
        err << "setCommand() requires one and only one entry in JointCommand.name when setCommand() is called.";
        RCS::Logger::log("gov.nasa.robonet.JointCommandRigid", Priority::ERROR, err.str());
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
        RCS::Logger::log("gov.nasa.robonet.JointCommandRigid", Priority::ERROR, err.str());
        throw runtime_error(err.str());
    }

    if (msg.name[0] != roboDynJoints[0])
    {
        stringstream err;
        err << "setCommand() received JointCommand message for joint [" << msg.name[0] << "]. It expected [" << roboDynJoints[0] << "].";
        RCS::Logger::log("gov.nasa.robonet.JointCommandRigid", Priority::ERROR, err.str());
        throw runtime_error(err.str());
    }

    if (not msg.desiredPosition.empty())
    {
        io.setFloat(desiredPositionCommandElement, msg.desiredPosition[0]-jointKinematicOffset);
    }
    if (not msg.desiredPositionVelocityLimit.empty())
    {
        io.setFloat(desiredPositionVelocityLimitCommandElement, msg.desiredPositionVelocityLimit[0]);
    }
}

nasa_r2_common_msgs::JointCapability JointCommandRigid::getCapability()
{
    return jointCapabilityMsg;
}
