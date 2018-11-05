#include "nasa_robodyn_mechanisms_core/JointCommandSeriesElastic.h"

using namespace std;
using namespace log4cpp;
namespace SU = StringUtilities;

JointCommandSeriesElastic::JointCommandSeriesElastic(const std::string& mechanism, IoFunctions ioFunctions)
    : JointCommandInterface(mechanism, ioFunctions),
      springConstant(0.0),
      gearStiffness(0.0),
      combinedStiffness(0.0),
      jointKinematicOffset(0.0),
      jointKinematicDirection(0.0),
      jointGearRatio(0.0),
      backEmfConstant(0.0),
      incEncNow(0),
      incEncPrev(0),
      incEncRef(0),
      encPosRef(0.0),
      hallEncNow(0),
      hallEncPrev(0),
      hallEncRef(0),
      hallPosRef(0.0),
      timeNow(0.0),
      timePrev(0.0),
      deltaTime(0.0),
      deltaTimeSec(0.0),
      deltaEncPos(0.0),
      commandRate(0.0),
      isInitialized(false),
      completeMessageSize(7)
{
    if (mechanism == "")
    {
        stringstream err;
        err << "Constructor requires mechanism be non-empty.";
        RCS::Logger::log("gov.nasa.robonet.JointCommandSeriesElastic", Priority::FATAL, err.str());
        throw invalid_argument(err.str());
    }

    // if (io.getFloat.empty()  or
        // io.setFloat.empty()  or
        // io.getInt32.empty()  or
        // io.getMotorCoeff.empty()  or
        // io.getCommandFile.empty())
    // {
        // stringstream err;
        // err << "Constructor requires 'io.getFloat', 'io.setFloat', 'io.getInt32', 'io.getMotorCoeff', and 'io.getCommandFile' be non-empty.";
        // RCS::Logger::log("gov.nasa.robonet.JointCommandSeriesElastic", Priority::FATAL, err.str());
        // throw invalid_argument(err.str());
    // }

    if (io.getFloat.empty())
    {
        stringstream err;
        err << "Constructor requires 'io.getFloat' be non-empty.";
        RCS::Logger::log("gov.nasa.robonet.JointCommandSeriesElastic", Priority::FATAL, err.str());
        throw invalid_argument(err.str());
    }
    if (io.setFloat.empty())
    {
        stringstream err;
        err << "Constructor requires 'io.setFloat' be non-empty.";
        RCS::Logger::log("gov.nasa.robonet.JointCommandSeriesElastic", Priority::FATAL, err.str());
        throw invalid_argument(err.str());
    }
    if (io.getInt32.empty())
    {
        stringstream err;
        err << "Constructor requires 'io.getInt32' be non-empty.";
        RCS::Logger::log("gov.nasa.robonet.JointCommandSeriesElastic", Priority::FATAL, err.str());
        throw invalid_argument(err.str());
    }
    if (io.setUInt16.empty())
    {
        stringstream err;
        err << "Constructor requires 'io.setUInt16' be non-empty.";
        RCS::Logger::log("gov.nasa.robonet.JointCommandSeriesElastic", Priority::FATAL, err.str());
        throw invalid_argument(err.str());
    }
    if (io.getMotorCoeff.empty())
    {
        stringstream err;
        err << "Constructor requires 'io.getMotorCoeff' be non-empty.";
        RCS::Logger::log("gov.nasa.robonet.JointCommandSeriesElastic", Priority::FATAL, err.str());
        throw invalid_argument(err.str());
    }
    if (io.hasBrainstemCoeff.empty())
    {
        stringstream err;
        err << "Constructor requires 'io.hasBrainstemCoeff' be non-empty.";
        RCS::Logger::log("gov.nasa.robonet.JointCommandSeriesElastic", Priority::FATAL, err.str());
        throw invalid_argument(err.str());
    }
    if (io.getBrainstemCoeff.empty())
    {
        stringstream err;
        err << "Constructor requires 'io.getBrainstemCoeff' be non-empty.";
        RCS::Logger::log("gov.nasa.robonet.JointCommandSeriesElastic", Priority::FATAL, err.str());
        throw invalid_argument(err.str());
    }
    if (io.getCommandFile.empty())
    {
        stringstream err;
        err << "Constructor requires 'io.getCommandFile' be non-empty.";
        RCS::Logger::log("gov.nasa.robonet.JointCommandSeriesElastic", Priority::FATAL, err.str());
        throw invalid_argument(err.str());
    }
    
    roboDynJoints.push_back(mechanism);
    motorName             = SU::getRoboDynEverythingButJoint(mechanism) + SU::TOKEN_DELIMITER + string("motor")             + SU::getNumber(SU::getRoboDynJoint(mechanism));
    encoderName           = SU::getRoboDynEverythingButJoint(mechanism) + SU::TOKEN_DELIMITER + string("encoder")           + SU::getNumber(SU::getRoboDynJoint(mechanism));
    jointCalculatedName   = SU::getRoboDynEverythingButJoint(mechanism) + SU::TOKEN_DELIMITER + string("jointCalculated")   + SU::getNumber(SU::getRoboDynJoint(mechanism));
    encoderCalculatedName = SU::getRoboDynEverythingButJoint(mechanism) + SU::TOKEN_DELIMITER + string("encoderCalculated") + SU::getNumber(SU::getRoboDynJoint(mechanism));
    hallsCalculatedName   = SU::getRoboDynEverythingButJoint(mechanism) + SU::TOKEN_DELIMITER + string("hallsCalculated")   + SU::getNumber(SU::getRoboDynJoint(mechanism));
    embeddedCommandName   = SU::getRoboDynEverythingButJoint(mechanism) + SU::TOKEN_DELIMITER + string("embeddedCommand")   + SU::getNumber(SU::getRoboDynJoint(mechanism));

    loadCoeffs();
}

JointCommandSeriesElastic::~JointCommandSeriesElastic()
{}

void JointCommandSeriesElastic::loadCoeffs()
{
    springConstant = 0.0;
    gearStiffness = 0.0;
    combinedStiffness = 0.0;
    jointKinematicOffset = 0.0;
    jointKinematicDirection = 0.0;
    jointGearRatio = 0.0;
    backEmfConstant = 0.0;
    incEncNow = 0;
    incEncPrev = 0;
    incEncRef = 0;
    encPosRef = 0.0;
    hallEncNow = 0;
    hallEncPrev = 0;
    hallEncRef = 0;
    hallPosRef = 0.0;
    timeNow = ros::Time(0.0);
    timePrev = ros::Time(0.0);
    deltaTime = ros::Duration(0.0);
    deltaTimeSec = 0.0;
    deltaEncPos = 0.0;
    commandRate = 0.0;
    isInitialized = false;

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
    completeMeasuredStateMsg.name[1] = motorName;
    completeMeasuredStateMsg.name[2] = encoderName;
    completeMeasuredStateMsg.name[3] = jointCalculatedName;
    completeMeasuredStateMsg.name[4] = encoderCalculatedName;
    completeMeasuredStateMsg.name[5] = hallsCalculatedName;
    completeMeasuredStateMsg.name[6] = embeddedCommandName;

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

    positionVals.resize(completeMessageSize, 0.0);
    velocityVals.resize(completeMessageSize, 0.0);
    effortVals.resize(completeMessageSize, 0.0);

    //! Parse parameter file
    TiXmlDocument file(parameterFile.c_str());
    bool loadOkay = file.LoadFile();
    if (!loadOkay)
    {
        stringstream err;
        err << "Failed to load file [" << parameterFile << "]";
        RCS::Logger::log("gov.nasa.robonet.JointCommandSeriesElastic", Priority::FATAL, err.str());
        throw runtime_error(err.str());
    }
    TiXmlHandle doc(&file);
    RCS::Logger::log("gov.nasa.robonet.JointCommandSeriesElastic", Priority::INFO, "CommandFile [" + parameterFile + "] successfully loaded.");

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
            //! joint
        jointPositionStatusElement =                 StringUtilities::makeFullyQualifiedRoboDynElement( roboDynJoints[0], ApiMap::getXmlElementValue(parametersElement, "JointPositionStatus"));
        jointVelocityStatusElement =                 StringUtilities::makeFullyQualifiedRoboDynElement( roboDynJoints[0], ApiMap::getXmlElementValue(parametersElement, "JointVelocityStatus"));
        jointCalculatedEffortElement =               StringUtilities::makeFullyQualifiedRoboDynElement( roboDynJoints[0], ApiMap::getXmlElementValue(parametersElement, "JointCalculatedEffort"));
            //! motor
        motorPositionStatusElement =                 StringUtilities::makeFullyQualifiedRoboDynElement( roboDynJoints[0], ApiMap::getXmlElementValue(parametersElement, "MotorPositionStatus"));
        motorVelocityStatusElement =                 StringUtilities::makeFullyQualifiedRoboDynElement( roboDynJoints[0], ApiMap::getXmlElementValue(parametersElement, "MotorVelocityStatus"));
        motorCurrentElement =                        StringUtilities::makeFullyQualifiedRoboDynElement( roboDynJoints[0], ApiMap::getXmlElementValue(parametersElement, "MotorCurrent"));
            //! encoder
        encoderPositionStatusElement =               StringUtilities::makeFullyQualifiedRoboDynElement( roboDynJoints[0], ApiMap::getXmlElementValue(parametersElement, "EncoderPositionStatus"));
        encoderVelocityStatusElement =               StringUtilities::makeFullyQualifiedRoboDynElement( roboDynJoints[0], ApiMap::getXmlElementValue(parametersElement, "EncoderVelocityStatus"));
            //! jointCalculated
        jointPositionFilteredStatusElement =         StringUtilities::makeFullyQualifiedRoboDynElement( roboDynJoints[0], ApiMap::getXmlElementValue(parametersElement, "JointPositionFilteredStatus"));
        motorPositionFilteredStatusElement =         StringUtilities::makeFullyQualifiedRoboDynElement( roboDynJoints[0], ApiMap::getXmlElementValue(parametersElement, "MotorPositionFilteredStatus"));
        springConstant =                             io.getMotorCoeff(StringUtilities::makeFullyQualifiedRoboDynElement( roboDynJoints[0], ApiMap::getXmlElementValue(parametersElement, "SpringConstant")));
        gearStiffness  =                             io.getMotorCoeff(StringUtilities::makeFullyQualifiedRoboDynElement( roboDynJoints[0], ApiMap::getXmlElementValue(parametersElement, "GearStiffness")));
            //! encoderCalculated
        encoderRawPositionStatusElement =            StringUtilities::makeFullyQualifiedRoboDynElement( roboDynJoints[0], ApiMap::getXmlElementValue(parametersElement, "EncoderRawPositionStatus"));
        encoderTimeElement =                         StringUtilities::makeFullyQualifiedRoboDynElement( roboDynJoints[0], ApiMap::getXmlElementValue(parametersElement, "EncoderTime"));
        encoderTimeoutElement =                      StringUtilities::makeFullyQualifiedRoboDynElement( roboDynJoints[0], ApiMap::getXmlElementValue(parametersElement, "EncoderTimeout"));
            //! hallsCalculated
        hallPositionStatusElement =                  StringUtilities::makeFullyQualifiedRoboDynElement( roboDynJoints[0], ApiMap::getXmlElementValue(parametersElement, "HallPositionStatus"));
        hallRawPositionStatusElement =               StringUtilities::makeFullyQualifiedRoboDynElement( roboDynJoints[0], ApiMap::getXmlElementValue(parametersElement, "HallRawPositionStatus"));
        hallTimeElement =                            StringUtilities::makeFullyQualifiedRoboDynElement( roboDynJoints[0], ApiMap::getXmlElementValue(parametersElement, "HallTime"));
        hallTimeoutElement =                         StringUtilities::makeFullyQualifiedRoboDynElement( roboDynJoints[0], ApiMap::getXmlElementValue(parametersElement, "HallTimeout"));
            //! embeddedCommand
        embeddedCommandPositionElement =             StringUtilities::makeFullyQualifiedRoboDynElement( roboDynJoints[0], ApiMap::getXmlElementValue(parametersElement, "EmbeddedCommandPosition"));
        embeddedCommandVelocityElement =             StringUtilities::makeFullyQualifiedRoboDynElement( roboDynJoints[0], ApiMap::getXmlElementValue(parametersElement, "EmbeddedCommandVelocity"));
        embeddedCommandEffortElement =               StringUtilities::makeFullyQualifiedRoboDynElement( roboDynJoints[0], ApiMap::getXmlElementValue(parametersElement, "EmbeddedCommandEffort"));
        //! Command Elements
        desiredPositionCommandElement =              StringUtilities::makeFullyQualifiedRoboDynElement( roboDynJoints[0], ApiMap::getXmlElementValue(parametersElement, "DesiredPositionCommand"));
        desiredPositionVelocityLimitCommandElement = StringUtilities::makeFullyQualifiedRoboDynElement( roboDynJoints[0], ApiMap::getXmlElementValue(parametersElement, "DesiredPositionVelocityLimitCommand"));
        feedForwardTorqueCommandElement =            StringUtilities::makeFullyQualifiedRoboDynElement( roboDynJoints[0], ApiMap::getXmlElementValue(parametersElement, "FeedForwardTorqueCommand"));
        proportionalGainCommandElement =             StringUtilities::makeFullyQualifiedRoboDynElement( roboDynJoints[0], ApiMap::getXmlElementValue(parametersElement, "ProportionalGainCommand"));
        derivativeGainCommandElement =               StringUtilities::makeFullyQualifiedRoboDynElement( roboDynJoints[0], ApiMap::getXmlElementValue(parametersElement, "DerivativeGainCommand"));
        integralGainCommandElement =                 StringUtilities::makeFullyQualifiedRoboDynElement( roboDynJoints[0], ApiMap::getXmlElementValue(parametersElement, "IntegralGainCommand"));
        positionLoopTorqueLimitCommandElement =      StringUtilities::makeFullyQualifiedRoboDynElement( roboDynJoints[0], ApiMap::getXmlElementValue(parametersElement, "PositionLoopTorqueLimitCommand"));
        positionLoopWindupLimitCommandElement =      StringUtilities::makeFullyQualifiedRoboDynElement( roboDynJoints[0], ApiMap::getXmlElementValue(parametersElement, "PositionLoopWindupLimitCommand"));
        torqueLoopVelocityLimitCommandElement =      StringUtilities::makeFullyQualifiedRoboDynElement( roboDynJoints[0], ApiMap::getXmlElementValue(parametersElement, "TorqueLoopVelocityLimitCommand"));
        //! Capability Elements
        jointKinematicOffset =                       io.getBrainstemCoeff(StringUtilities::makeFullyQualifiedRoboDynElement( roboDynJoints[0], ApiMap::getXmlElementValue(parametersElement, "APSKinematicOffset")));
        jointKinematicDirection =                    io.getMotorCoeff(StringUtilities::makeFullyQualifiedRoboDynElement( roboDynJoints[0], ApiMap::getXmlElementValue(parametersElement, "JointKinematicDir")));
        jointCapabilityMsg.positionLimitMax[0] =     io.getMotorCoeff(StringUtilities::makeFullyQualifiedRoboDynElement( roboDynJoints[0], ApiMap::getXmlElementValue(parametersElement, "PositionLimitMax")))
                + jointKinematicOffset;
        jointCapabilityMsg.positionLimitMin[0] =     io.getMotorCoeff(StringUtilities::makeFullyQualifiedRoboDynElement( roboDynJoints[0], ApiMap::getXmlElementValue(parametersElement, "PositionLimitMin")))
                + jointKinematicOffset;
        jointCapabilityMsg.torqueLimit[0] =          io.getMotorCoeff(StringUtilities::makeFullyQualifiedRoboDynElement( roboDynJoints[0], ApiMap::getXmlElementValue(parametersElement, "TorqueLimit")));
        jointGearRatio =                             io.getMotorCoeff(StringUtilities::makeFullyQualifiedRoboDynElement( roboDynJoints[0], ApiMap::getXmlElementValue(parametersElement, "JointGearRatio")));
        backEmfConstant =                            io.getMotorCoeff(StringUtilities::makeFullyQualifiedRoboDynElement( roboDynJoints[0], ApiMap::getXmlElementValue(parametersElement, "BackEMFConstant")));

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
        RCS::Logger::log("gov.nasa.robonet.JointCommandSeriesElastic", Priority::ERROR, err.str());
        throw runtime_error(err.str());
    }

    // Initialize previous values
    incEncPrev = io.getInt32(encoderRawPositionStatusElement);
    timePrev   = ros::Time::now();
    
    //! Needed to use the constructor version of make_shared because of the number of arguments in EncoderStateCalculator's constructor.
    motorEncoderStateCalculator = boost::make_shared<MotorEncoderStateCalculator>(MotorEncoderStateCalculator(jointGearRatio,
                                                                                                              io.getMotorCoeff(    StringUtilities::makeFullyQualifiedRoboDynElement( roboDynJoints[0], ApiMap::getXmlElementValue(parametersElement, "PwmFrequency"))),
                                                                                                              io.getMotorCoeff(    StringUtilities::makeFullyQualifiedRoboDynElement( roboDynJoints[0], ApiMap::getXmlElementValue(parametersElement, "PwmDeadTime"))),
                                                                                                              io.getBrainstemCoeff(StringUtilities::makeFullyQualifiedRoboDynElement( roboDynJoints[0], ApiMap::getXmlElementValue(parametersElement, "BusVoltage"))),
                                                                                                              io.getMotorCoeff(    StringUtilities::makeFullyQualifiedRoboDynElement( roboDynJoints[0], ApiMap::getXmlElementValue(parametersElement, "MotorCurrentLimit"))),
                                                                                                              io.getMotorCoeff(    StringUtilities::makeFullyQualifiedRoboDynElement( roboDynJoints[0], ApiMap::getXmlElementValue(parametersElement, "PhaseResistance"))),
                                                                                                              backEmfConstant,
                                                                                                              jointKinematicDirection,
                                                                                                              io.getMotorCoeff(    StringUtilities::makeFullyQualifiedRoboDynElement( roboDynJoints[0], ApiMap::getXmlElementValue(parametersElement, "EncoderMountingDir"))),
                                                                                                              io.getMotorCoeff(    StringUtilities::makeFullyQualifiedRoboDynElement( roboDynJoints[0], ApiMap::getXmlElementValue(parametersElement, "EncoderCountsPerRevolution"))),
                                                                                                              io.getMotorCoeff(    StringUtilities::makeFullyQualifiedRoboDynElement( roboDynJoints[0], ApiMap::getXmlElementValue(parametersElement, "EncoderGlitchScalar"))),
                                                                                                              io.getMotorCoeff(    StringUtilities::makeFullyQualifiedRoboDynElement( roboDynJoints[0], ApiMap::getXmlElementValue(parametersElement, "EncoderVelocityBlendStop"))),
                                                                                                              io.getMotorCoeff(    StringUtilities::makeFullyQualifiedRoboDynElement( roboDynJoints[0], ApiMap::getXmlElementValue(parametersElement, "EncoderVelocityBlendStart"))),
                                                                                                              io.getBrainstemCoeff(StringUtilities::makeFullyQualifiedRoboDynElement( roboDynJoints[0], ApiMap::getXmlElementValue(parametersElement, "EncoderVelocityAlpha")))));
    hallsEncoderStateCalculator = boost::make_shared<HallsEncoderStateCalculator>(HallsEncoderStateCalculator(jointGearRatio,
                                                                                                              io.getMotorCoeff(    StringUtilities::makeFullyQualifiedRoboDynElement( roboDynJoints[0], ApiMap::getXmlElementValue(parametersElement, "PwmFrequency"))),
                                                                                                              io.getMotorCoeff(    StringUtilities::makeFullyQualifiedRoboDynElement( roboDynJoints[0], ApiMap::getXmlElementValue(parametersElement, "PwmDeadTime"))),
                                                                                                              io.getBrainstemCoeff(StringUtilities::makeFullyQualifiedRoboDynElement( roboDynJoints[0], ApiMap::getXmlElementValue(parametersElement, "BusVoltage"))),
                                                                                                              io.getMotorCoeff(    StringUtilities::makeFullyQualifiedRoboDynElement( roboDynJoints[0], ApiMap::getXmlElementValue(parametersElement, "MotorCurrentLimit"))),
                                                                                                              io.getMotorCoeff(    StringUtilities::makeFullyQualifiedRoboDynElement( roboDynJoints[0], ApiMap::getXmlElementValue(parametersElement, "PhaseResistance"))),
                                                                                                              backEmfConstant,
                                                                                                              jointKinematicDirection,
                                                                                                              1,
                                                                                                              io.getBrainstemCoeff(StringUtilities::makeFullyQualifiedRoboDynElement( roboDynJoints[0], ApiMap::getXmlElementValue(parametersElement, "MotorPoleCount"))),
                                                                                                              io.getMotorCoeff(    StringUtilities::makeFullyQualifiedRoboDynElement( roboDynJoints[0], ApiMap::getXmlElementValue(parametersElement, "EncoderGlitchScalar"))),
                                                                                                              io.getBrainstemCoeff(StringUtilities::makeFullyQualifiedRoboDynElement( roboDynJoints[0], ApiMap::getXmlElementValue(parametersElement, "HallVelocityBlendStop"))),
                                                                                                              io.getBrainstemCoeff(StringUtilities::makeFullyQualifiedRoboDynElement( roboDynJoints[0], ApiMap::getXmlElementValue(parametersElement, "HallVelocityBlendStart"))),
                                                                                                              io.getBrainstemCoeff(StringUtilities::makeFullyQualifiedRoboDynElement( roboDynJoints[0], ApiMap::getXmlElementValue(parametersElement, "EncoderVelocityAlpha")))));

    combinedStiffness = 1.0 / ((1.0 / springConstant) + (1.0 / gearStiffness));
}

void JointCommandSeriesElastic::updateMeasuredState(nasa_r2_common_msgs::JointControlData msg)
{
    // If coeffs are not loaded, override with zeros RDEV-1075
    if (msg.coeffState.state == nasa_r2_common_msgs::JointControlCoeffState::LOADED)
    {
        //! encoderCalculated and hallsCalculated initialization
        timeNow         = ros::Time::now();
        incEncNow       = io.getInt32(encoderRawPositionStatusElement);
        hallEncNow      = io.getInt32(hallRawPositionStatusElement);
        if (not isInitialized)
        {
            isInitialized = true;
            incEncRef     = incEncNow;
            encPosRef     = io.getFloat(motorPositionStatusElement) - ((io.getFloat(jointCalculatedEffortElement) * -1) / gearStiffness);
            hallEncRef    = hallEncNow;
            hallPosRef    = io.getFloat(motorPositionStatusElement) - ((io.getFloat(jointCalculatedEffortElement) * -1) / gearStiffness);
            timePrev      = ros::Time::now();
        }
        commandRate     = 1 / ((timeNow - timePrev).toSec());
        timePrev        = timeNow;

        //! joint
        positionVals[0] = io.getFloat(jointPositionStatusElement) + jointKinematicOffset;
        velocityVals[0] = io.getFloat(jointVelocityStatusElement);
        effortVals[0]   = io.getFloat(jointCalculatedEffortElement) * -1; //! The motor drive sends up motor minus joint for now, so we have to negate that

        //! motor
        positionVals[1] = io.getFloat(motorPositionStatusElement) + jointKinematicOffset;
        velocityVals[1] = io.getFloat(motorVelocityStatusElement);
        effortVals[1]   = io.getFloat(motorCurrentElement) * backEmfConstant * jointGearRatio; //! jointGearRatio to get into joint space

        //! encoder
        positionVals[2] = io.getFloat(encoderPositionStatusElement) + jointKinematicOffset;
        velocityVals[2] = io.getFloat(encoderVelocityStatusElement) / jointGearRatio; //! jointGearRatio to get into joint space

        //! jointCalculated
        effortVals[3]   = (io.getFloat(jointPositionFilteredStatusElement) - io.getFloat(motorPositionFilteredStatusElement)) * springConstant;  //! APS2 - APS1

        //! encoderCalculated
        positionVals[4] = motorEncoderStateCalculator->getPosition(encPosRef, incEncRef, incEncNow) + jointKinematicOffset;
        velocityVals[4] = motorEncoderStateCalculator->getVelocity(io.getInt32(encoderTimeElement), io.getInt32(encoderTimeoutElement), incEncNow, commandRate) / jointGearRatio; //! jointGearRatio to get into joint space
        effortVals[4]   = (io.getFloat(jointPositionFilteredStatusElement) - io.getFloat(encoderPositionStatusElement)) * combinedStiffness;  //! APS2 - enc

        //! hallsCalculated
        positionVals[5] = hallsEncoderStateCalculator->getPosition(hallPosRef, hallEncRef, hallEncNow) + jointKinematicOffset;
        velocityVals[5] = hallsEncoderStateCalculator->getVelocity(io.getInt32(hallTimeElement), io.getInt32(hallTimeoutElement), hallEncNow, commandRate) / jointGearRatio; //! jointGearRatio to get into joint space
        //effortVals[5]   = (io.getFloat(motorPositionFilteredStatusElement) - io.getFloat(encoderPositionStatusElement)) * gearStiffness;  //! APS1 - enc

        //! embeddedCommand
        positionVals[6] = io.getFloat(embeddedCommandPositionElement) + jointKinematicOffset;
        velocityVals[6] = io.getFloat(embeddedCommandVelocityElement);
        effortVals[6]   = io.getFloat(embeddedCommandEffortElement);
    }
    else
    {
        isInitialized = false;
        positionVals.assign(completeMessageSize, 0.0);
        velocityVals.assign(completeMessageSize, 0.0);
        effortVals.assign(completeMessageSize, 0.0);
    }
}

sensor_msgs::JointState JointCommandSeriesElastic::getSimpleMeasuredState()
{
    simpleMeasuredStateMsg.header.stamp = ros::Time::now();

    simpleMeasuredStateMsg.position[0] = positionVals[0];
    simpleMeasuredStateMsg.velocity[0] = velocityVals[0];
    simpleMeasuredStateMsg.effort[0]   = effortVals[0];

    return simpleMeasuredStateMsg;
}

sensor_msgs::JointState JointCommandSeriesElastic::getCompleteMeasuredState()
{
    completeMeasuredStateMsg.header.stamp = ros::Time::now();

    completeMeasuredStateMsg.position = positionVals;
    completeMeasuredStateMsg.velocity = velocityVals;
    completeMeasuredStateMsg.effort   = effortVals;

    return completeMeasuredStateMsg;
}

void JointCommandSeriesElastic::setFaultState()
{
}

nasa_r2_common_msgs::JointCommand JointCommandSeriesElastic::getCommandedState()
{
    commandedStateMsg.header.stamp = ros::Time::now();

    commandedStateMsg.desiredPosition[0]              = io.getFloat(desiredPositionCommandElement)+jointKinematicOffset;
    commandedStateMsg.desiredPositionVelocityLimit[0] = io.getFloat(desiredPositionVelocityLimitCommandElement);
    commandedStateMsg.feedForwardTorque[0]            = io.getFloat(feedForwardTorqueCommandElement);
    commandedStateMsg.proportionalGain[0]             = io.getFloat(proportionalGainCommandElement);
    commandedStateMsg.derivativeGain[0]               = io.getFloat(derivativeGainCommandElement);
    commandedStateMsg.integralGain[0]                 = io.getFloat(integralGainCommandElement);
    commandedStateMsg.positionLoopTorqueLimit[0]      = io.getFloat(positionLoopTorqueLimitCommandElement);
    commandedStateMsg.positionLoopWindupLimit[0]      = io.getFloat(positionLoopWindupLimitCommandElement);
    // commandedStateMsg.torqueLoopVelocityLimit[0]      = io.getFloat(torqueLoopVelocityLimitCommandElement);

    return commandedStateMsg;
}

void JointCommandSeriesElastic::setCommand(nasa_r2_common_msgs::JointCommand msg, nasa_r2_common_msgs::JointControlData)
{
    if (msg.name.size() != 1)
    {
        stringstream err;
        err << "setCommand() requires one and only one entry in JointCommand.name when setCommand() is called.";
        RCS::Logger::log("gov.nasa.robonet.JointCommandSeriesElastic", Priority::ERROR, err.str());
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
        RCS::Logger::log("gov.nasa.robonet.JointCommandSeriesElastic", Priority::ERROR, err.str());
        throw runtime_error(err.str());
    }

    if (msg.name[0] != roboDynJoints[0])
    {
        stringstream err;
        err << "setCommand() received JointCommand message for joint [" << msg.name[0] << "]. It expected [" << roboDynJoints[0] << "].";
        RCS::Logger::log("gov.nasa.robonet.JointCommandSeriesElastic", Priority::ERROR, err.str());
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

nasa_r2_common_msgs::JointCapability JointCommandSeriesElastic::getCapability()
{
    return jointCapabilityMsg;
}
