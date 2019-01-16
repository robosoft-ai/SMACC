#include "nasa_robodyn_mechanisms_core/JointCommandWrist.h"
#include "nasa_r2_common_msgs/JointControlDataArray.h"

using namespace std;
using namespace log4cpp;
using namespace Eigen;

JointCommandWrist::JointCommandWrist(const std::string& mechanism, IoFunctions ioFunctions)
    : JointCommandInterface(mechanism, ioFunctions)
    , desiredPitchVel(0.f)
    , desiredYawVel(0.f)
    , sliderFilterInitialized(false)
    , hallFilterInitialized(false)
    , prevPosInitialized(false)
    , calFailureReported(false)
    , completeMessageSize(12)
{
    if (mechanism == "")
    {
        stringstream err;
        err << "Constructor requires mechanism be non-empty.";
        RCS::Logger::log("gov.nasa.robonet.JointCommandWrist", Priority::FATAL, err.str());
        throw invalid_argument(err.str());
    }

    if (io.setInt16.empty() or
        io.getInt16.empty() or
        io.getUInt16.empty() or
        io.getBrainstemCoeff.empty() or
        io.getMotorCoeff.empty() or
        io.hasLiveCoeff.empty() or
        io.getLiveCoeff.empty() or
        io.setLiveCoeff.empty() or
        io.getJointNames.empty() or
        io.getActuatorNames.empty() or
        io.getCommandFile.empty())
    {
        stringstream err;
        err << "Constructor requires 'io.setInt16', 'io.getInt16', 'io.getUint16', 'io.getBrainstemCoeff', 'io.getMotorCoeff', 'io.hasLiveCoeff', 'io.getLiveCoeff', 'io.setLiveCoeff', 'io.getJointNames', 'io.getActuatorNames', and 'io.getCommandFile' be non-empty.";
        RCS::Logger::log("gov.nasa.robonet.JointCommandWrist", Priority::FATAL, err.str());
        throw invalid_argument(err.str());
    }


    roboDynJoints    = io.getJointNames(mechanism);
    roboDynActuators = io.getActuatorNames(mechanism);


    if(roboDynJoints.size() != 2 or
       roboDynActuators.size() != 2)
    {
        stringstream err;
        err << "Constructor requires roboDynJoints and roboDynActuators to have 2 values";
        RCS::Logger::log("gov.nasa.robonet.JointCommandWrist", Priority::FATAL, err.str());
        throw invalid_argument(err.str());

    }
    if(roboDynJoints[0].find("pitch", 0) == string::npos)
    {
        stringstream err;
        err << "Expected pitch element, got "<<roboDynJoints[0]<<"."<<std::endl;
        RCS::Logger::log("gov.nasa.robonet.JointCommandWrist", Priority::FATAL, err.str());
        throw invalid_argument(err.str());
    }

    if(roboDynJoints[1].find("yaw", 0) == string::npos)
    {
        stringstream err;
        err << "Expected yaw element, got "<<roboDynJoints[1]<<"."<<std::endl;
        RCS::Logger::log("gov.nasa.robonet.JointCommandWrist", Priority::FATAL, err.str());
        throw invalid_argument(err.str());
    }

    if(roboDynActuators[0].find("thumbside",0) == string::npos)
    {
        stringstream err;
        err << "Expected thumbside element, got "<<roboDynActuators[0]<<"."<<std::endl;
        RCS::Logger::log("gov.nasa.robonet.JointCommandWrist", Priority::FATAL, err.str());
        throw invalid_argument(err.str());
    }

    if(roboDynActuators[1].find("littleside",0) == string::npos)
    {
        stringstream err;
        err << "Expected littlside element, got "<<roboDynActuators[1]<<"."<<std::endl;
        RCS::Logger::log("gov.nasa.robonet.JointCommandWrist", Priority::FATAL, err.str());
        throw invalid_argument(err.str());
    }

    wrist.reset(new WristMechanism());
    wristFault.reset(new WristMechanism());
    q1Controller.reset(new MultiLoopController());
    q2Controller.reset(new MultiLoopController());

    loadCoeffs();
}

JointCommandWrist::~JointCommandWrist()
{

}

void JointCommandWrist::loadCoeffs()
{
    io.setLiveCoeff(nameCoeffState, nasa_r2_common_msgs::JointControlCoeffState::NOTLOADED);

    desiredPitchVel = 0.f;
    desiredYawVel = 0.f;
    sliderFilterInitialized = false;
    hallFilterInitialized = false;
    prevPosInitialized = false;
    calFailureReported = false;

    std::string parameterFile = io.getCommandFile(mechanism);

    //! Initialize live coeff names
    //nameCalibrationState      = StringUtilities::makeFullyQualifiedRoboDynElement(mechanism, "CalibrationState");
    namePitchLimit            = StringUtilities::makeFullyQualifiedRoboDynElement(mechanism, "PitchLimit");
    nameYawLimit              = StringUtilities::makeFullyQualifiedRoboDynElement(mechanism, "YawLimit");
    nameLittlesideSliderLimit = StringUtilities::makeFullyQualifiedRoboDynElement(mechanism, "LittlesideSliderLimit");
    nameThumbsideSliderLimit  = StringUtilities::makeFullyQualifiedRoboDynElement(mechanism, "ThumbsideSliderLimit");
    nameSensorError           = StringUtilities::makeFullyQualifiedRoboDynElement(mechanism, "SensorError");
    nameSliderDiffError       = StringUtilities::makeFullyQualifiedRoboDynElement(mechanism, "SliderDiffError");
    nameUseHalls              = StringUtilities::makeFullyQualifiedRoboDynElement(mechanism, "UseHalls");
    nameCalibrationMode       = StringUtilities::makeFullyQualifiedRoboDynElement(mechanism, "CalibrationMode");
    nameCoeffState            = StringUtilities::makeFullyQualifiedRoboDynElement(mechanism, "CoeffState");

    //! Initialize messages
    simpleMeasuredStateMsg.name.resize(4);
    simpleMeasuredStateMsg.position.resize(4, 0.);
    simpleMeasuredStateMsg.velocity.resize(4, 0.);
    simpleMeasuredStateMsg.effort.resize(4, 0.);

    simpleMeasuredStateMsg.name[0] = roboDynJoints[0];
    simpleMeasuredStateMsg.name[1] = roboDynJoints[1];
    simpleMeasuredStateMsg.name[2] = roboDynActuators[0];
    simpleMeasuredStateMsg.name[3] = roboDynActuators[1];

    completeMeasuredStateMsg.name.resize(completeMessageSize);
    completeMeasuredStateMsg.position.resize(completeMessageSize, 0.);
    completeMeasuredStateMsg.velocity.resize(completeMessageSize, 0.);
    completeMeasuredStateMsg.effort.resize(completeMessageSize, 0.);

    completeMeasuredStateMsg.name[0] = roboDynJoints[0];
    completeMeasuredStateMsg.name[1] = roboDynJoints[1];
    completeMeasuredStateMsg.name[2] = roboDynActuators[0];
    completeMeasuredStateMsg.name[3] = roboDynActuators[1];
    completeMeasuredStateMsg.name[4] = roboDynJoints.at(0) +  StringUtilities::TOKEN_DELIMITER + string("halls");
    completeMeasuredStateMsg.name[5] = roboDynJoints.at(1) +  StringUtilities::TOKEN_DELIMITER + string("halls");
    completeMeasuredStateMsg.name[6] = roboDynActuators.at(0) + StringUtilities::TOKEN_DELIMITER + string("encoder");
    completeMeasuredStateMsg.name[7] = roboDynActuators.at(1) + StringUtilities::TOKEN_DELIMITER + string("encoder");
    completeMeasuredStateMsg.name[8] = roboDynJoints.at(0) +  StringUtilities::TOKEN_DELIMITER + string("embeddedCommand");
    completeMeasuredStateMsg.name[9] = roboDynJoints.at(1) +  StringUtilities::TOKEN_DELIMITER + string("embeddedCommand");
    completeMeasuredStateMsg.name[10] = roboDynActuators.at(0) + StringUtilities::TOKEN_DELIMITER + string("embeddedCommand");
    completeMeasuredStateMsg.name[11] = roboDynActuators.at(1) + StringUtilities::TOKEN_DELIMITER + string("embeddedCommand");


    positionVals.resize(completeMessageSize, 0.);
    velocityVals.resize(completeMessageSize, 0.);

    commandedStateMsg.name.resize(2);
    commandedStateMsg.desiredPosition.resize(2, 0.);
    commandedStateMsg.desiredPositionVelocityLimit.resize(2, 0.);
    commandedStateMsg.feedForwardTorque.resize(2, 0.);
    commandedStateMsg.proportionalGain.resize(2, 0.);
    commandedStateMsg.derivativeGain.resize(2, 0.);
    commandedStateMsg.integralGain.resize(2, 0.);
    commandedStateMsg.positionLoopTorqueLimit.resize(2, 0.);
    commandedStateMsg.positionLoopWindupLimit.resize(2, 0.);
    commandedStateMsg.torqueLoopVelocityLimit.resize(2, 0.);

    commandedStateMsg.name[0] = roboDynJoints[0];
    commandedStateMsg.name[1] = roboDynJoints[1];
    commandedStateMsg.desiredPosition[0] = 0;
    commandedStateMsg.desiredPosition[1] = 0;

    jointCapabilityMsg.name.resize(2);
    jointCapabilityMsg.positionLimitMax.resize(2, 0.);
    jointCapabilityMsg.positionLimitMin.resize(2, 0.);
    jointCapabilityMsg.torqueLimit.resize(2, 0.);

    jointCapabilityMsg.name[0] = roboDynJoints[0];
    jointCapabilityMsg.name[1] = roboDynJoints[1];

    //! Parse parameter file
    TiXmlDocument file(parameterFile.c_str());
    bool loadOkay = file.LoadFile();
    if (!loadOkay)
    {
        stringstream err;
        err << "Failed to load file [" << parameterFile << "]";
        RCS::Logger::log("gov.nasa.robonet.JointCommandWrist", Priority::FATAL, err.str());
        throw runtime_error(err.str());
    }
    TiXmlHandle doc(&file);
    RCS::Logger::log("gov.nasa.robonet.JointCommandWrist", Priority::INFO, "CommandFile [" + parameterFile + "] successfully loaded.");

    // Check for ApiMap
    TiXmlHandle parametersElement(doc.FirstChildElement("ApiMap"));

    if (parametersElement.ToElement())
    {

        //! Status Elements
        encoderThumbPositionStatusElement =        StringUtilities::makeFullyQualifiedRoboDynElement( roboDynActuators[0],    ApiMap::getXmlElementValue(parametersElement, "EncoderThumbSidePositionStatus"));
        encoderLittlePositionStatusElement =       StringUtilities::makeFullyQualifiedRoboDynElement( roboDynActuators[1],    ApiMap::getXmlElementValue(parametersElement, "EncoderLittleSidePositionStatus"));
        hallsPitchStatusElement =                  StringUtilities::makeFullyQualifiedRoboDynElement( roboDynJoints[0],       ApiMap::getXmlElementValue(parametersElement, "HallsPitchStatus"));
        hallsYawStatusElement =                    StringUtilities::makeFullyQualifiedRoboDynElement( roboDynJoints[1],       ApiMap::getXmlElementValue(parametersElement, "HallsYawStatus"));

        //! Command Elements
        desiredMotComThumbCommandElement =         StringUtilities::makeFullyQualifiedRoboDynElement( roboDynActuators[0], ApiMap::getXmlElementValue(parametersElement, "DesiredMotComThumbSideCommand"));
        desiredMotComLittleCommandElement =        StringUtilities::makeFullyQualifiedRoboDynElement( roboDynActuators[1], ApiMap::getXmlElementValue(parametersElement, "DesiredMotComLittleSideCommand"));

        //! Multi-loop parameters
        double bypass =         io.getMotorCoeff(StringUtilities::makeFullyQualifiedRoboDynElement( mechanism,  ApiMap::getXmlElementValue(parametersElement, "PositionLoopBypass")));
        sliderMinPosition =     io.getMotorCoeff(StringUtilities::makeFullyQualifiedRoboDynElement( mechanism,  ApiMap::getXmlElementValue(parametersElement, "SliderMinPositionCommand")));
        sliderMaxPosition =     io.getMotorCoeff(StringUtilities::makeFullyQualifiedRoboDynElement( mechanism,  ApiMap::getXmlElementValue(parametersElement, "SliderMaxPositionCommand")));
        double pKp =            io.getMotorCoeff(StringUtilities::makeFullyQualifiedRoboDynElement( mechanism,  ApiMap::getXmlElementValue(parametersElement, "SliderKpGainCommand")));

        q1Controller->setPositionLoopParameters(sliderMinPosition, sliderMaxPosition, pKp, bypass <= 0 ? false : true);
        q2Controller->setPositionLoopParameters(sliderMinPosition, sliderMaxPosition, pKp, bypass <= 0 ? false : true);
        
        // torque loop is always bypassed for wrists
        q1Controller->setTorqueLoopParameters(0., 0., 0., 0., true);
        q2Controller->setTorqueLoopParameters(0., 0., 0., 0., true);

        bypass =                io.getMotorCoeff(StringUtilities::makeFullyQualifiedRoboDynElement( mechanism,  ApiMap::getXmlElementValue(parametersElement, "VelocityLoopBypass")));
        double minVelocity =    io.getMotorCoeff(StringUtilities::makeFullyQualifiedRoboDynElement( mechanism,  ApiMap::getXmlElementValue(parametersElement, "SliderMinVelocityCommand" )));
        double maxVelocity =    io.getMotorCoeff(StringUtilities::makeFullyQualifiedRoboDynElement( mechanism,  ApiMap::getXmlElementValue(parametersElement, "SliderMaxVelocityCommand" )));
        double vKp =            io.getMotorCoeff(StringUtilities::makeFullyQualifiedRoboDynElement( mechanism,  ApiMap::getXmlElementValue(parametersElement, "SliderVelocityKpGainCommand" )));
        double vKi =            io.getMotorCoeff(StringUtilities::makeFullyQualifiedRoboDynElement( mechanism,  ApiMap::getXmlElementValue(parametersElement, "SliderVelocityKiGainCommand" )));
        double windup =         io.getMotorCoeff(StringUtilities::makeFullyQualifiedRoboDynElement( mechanism,  ApiMap::getXmlElementValue(parametersElement, "SliderIntegratorWindupLimit" )));

        q1Controller->setVelocityLoopParameters(minVelocity, maxVelocity, vKp, vKi, windup, bypass <= 0 ? false : true);
        q2Controller->setVelocityLoopParameters(minVelocity, maxVelocity, vKp, vKi, windup, bypass <= 0 ? false : true);

        bypass =                io.getMotorCoeff(StringUtilities::makeFullyQualifiedRoboDynElement( mechanism,  ApiMap::getXmlElementValue(parametersElement, "CurrentLoopBypass")));
        double minCurrent =     io.getMotorCoeff(StringUtilities::makeFullyQualifiedRoboDynElement( mechanism,  ApiMap::getXmlElementValue(parametersElement, "SliderMinCurrentCommand" )));
        double maxCurrent =     io.getMotorCoeff(StringUtilities::makeFullyQualifiedRoboDynElement( mechanism,  ApiMap::getXmlElementValue(parametersElement, "SliderMaxCurrentCommand" )));
        double maxDutyCycle =   io.getMotorCoeff(StringUtilities::makeFullyQualifiedRoboDynElement( mechanism,  ApiMap::getXmlElementValue(parametersElement, "SliderMaxDutyCycleCommand" )));

        q1Controller->setCurrentLoopParameters(minCurrent, maxCurrent, maxDutyCycle, bypass <= 0 ? false : true);
        q2Controller->setCurrentLoopParameters(minCurrent, maxCurrent, maxDutyCycle, bypass <= 0 ? false : true);

        double motorViscousDampingComp =    io.getMotorCoeff(StringUtilities::makeFullyQualifiedRoboDynElement( mechanism,  ApiMap::getXmlElementValue(parametersElement, "MotorViscousDampingCompensation" )));
        double motorInertia =               io.getMotorCoeff(StringUtilities::makeFullyQualifiedRoboDynElement( mechanism,  ApiMap::getXmlElementValue(parametersElement, "MotorInertia" )));
        double motorTorqueConst =           io.getMotorCoeff(StringUtilities::makeFullyQualifiedRoboDynElement( mechanism,  ApiMap::getXmlElementValue(parametersElement, "MotorTorqueConstant" )));
        double motorPhaseRes =              io.getMotorCoeff(StringUtilities::makeFullyQualifiedRoboDynElement( mechanism,  ApiMap::getXmlElementValue(parametersElement, "MotorPhaseResistance" )));
        double motorBackEMF =               io.getMotorCoeff(StringUtilities::makeFullyQualifiedRoboDynElement( mechanism,  ApiMap::getXmlElementValue(parametersElement, "MotorBackEMFConstant" )));
        double motorInductance =            io.getMotorCoeff(StringUtilities::makeFullyQualifiedRoboDynElement( mechanism,  ApiMap::getXmlElementValue(parametersElement, "MotorInductance" )));
        double motionRatio =                io.getMotorCoeff(StringUtilities::makeFullyQualifiedRoboDynElement( mechanism,  ApiMap::getXmlElementValue(parametersElement, "MotionRatio" )));
        double PWMFreq =                    io.getMotorCoeff(StringUtilities::makeFullyQualifiedRoboDynElement( mechanism,  ApiMap::getXmlElementValue(parametersElement, "PWMFrequency" )));
        double bridgeDeadTime =             io.getMotorCoeff(StringUtilities::makeFullyQualifiedRoboDynElement( mechanism,  ApiMap::getXmlElementValue(parametersElement, "BridgeDeadTime" )));
        double bridgeSwitchTime =           io.getMotorCoeff(StringUtilities::makeFullyQualifiedRoboDynElement( mechanism,  ApiMap::getXmlElementValue(parametersElement, "BridgeSwitchTime" )));

        q1Controller->setHardwareParameters(motorViscousDampingComp, motorInertia, motorTorqueConst, motorPhaseRes, motorBackEMF, motorInductance, motionRatio, PWMFreq, bridgeDeadTime, bridgeSwitchTime);
        q2Controller->setHardwareParameters(motorViscousDampingComp, motorInertia, motorTorqueConst, motorPhaseRes, motorBackEMF, motorInductance, motionRatio, PWMFreq, bridgeDeadTime, bridgeSwitchTime);

        busVoltage =            io.getBrainstemCoeff(StringUtilities::makeFullyQualifiedRoboDynElement( mechanism,  ApiMap::getXmlElementValue(parametersElement, "BusVoltage")));

        // filter
        positionAlpha = io.getMotorCoeff(StringUtilities::makeFullyQualifiedRoboDynElement( mechanism,  ApiMap::getXmlElementValue(parametersElement, "PositionAlpha")));
        timestep      = io.getMotorCoeff(StringUtilities::makeFullyQualifiedRoboDynElement( mechanism,  ApiMap::getXmlElementValue(parametersElement, "Timestep")));

        // "embedded" smoothing
        EmbeddedSmoother::Settings smoothSettings;
        if (timestep <= 0.)
        {
            throw std::runtime_error("invalid timestep");
        }
        smoothSettings.timestep = timestep;
        smoothSettings.minVel = io.getMotorCoeff(StringUtilities::makeFullyQualifiedRoboDynElement( mechanism,  ApiMap::getXmlElementValue(parametersElement, "VelocityMinPitch")));
        smoothSettings.maxAcc = io.getMotorCoeff(StringUtilities::makeFullyQualifiedRoboDynElement( mechanism,  ApiMap::getXmlElementValue(parametersElement, "AccelerationMaxPitch")));
        smoothSettings.accGain = io.getMotorCoeff(StringUtilities::makeFullyQualifiedRoboDynElement( mechanism,  ApiMap::getXmlElementValue(parametersElement, "AccelerationGainPitch")));
        smootherPitch.reset(new EmbeddedSmoother(smoothSettings));
        smoothSettings.minVel = io.getMotorCoeff(StringUtilities::makeFullyQualifiedRoboDynElement( mechanism,  ApiMap::getXmlElementValue(parametersElement, "VelocityMinYaw")));
        smoothSettings.maxAcc = io.getMotorCoeff(StringUtilities::makeFullyQualifiedRoboDynElement( mechanism,  ApiMap::getXmlElementValue(parametersElement, "AccelerationMaxYaw")));
        smoothSettings.accGain = io.getMotorCoeff(StringUtilities::makeFullyQualifiedRoboDynElement( mechanism,  ApiMap::getXmlElementValue(parametersElement, "AccelerationGainYaw")));
        smootherYaw.reset(new EmbeddedSmoother(smoothSettings));

        // multiloop params
        q1Controller->setLoopRate(1/timestep);
        q2Controller->setLoopRate(1/timestep);

        //! Capability Elements
        Vector3f a, a0, b, c, d, d0, pitch, yaw, m, n;

        a(0) =                  io.getMotorCoeff(StringUtilities::makeFullyQualifiedRoboDynElement( mechanism,  ApiMap::getXmlElementValue(parametersElement, "Ax")));
        a(1) =                  io.getMotorCoeff(StringUtilities::makeFullyQualifiedRoboDynElement( mechanism,  ApiMap::getXmlElementValue(parametersElement, "Ay")));
        a(2) =                  io.getMotorCoeff(StringUtilities::makeFullyQualifiedRoboDynElement( mechanism,  ApiMap::getXmlElementValue(parametersElement, "Az")));

        a0(0) =                 io.getMotorCoeff(StringUtilities::makeFullyQualifiedRoboDynElement( mechanism,  ApiMap::getXmlElementValue(parametersElement, "A0x")));
        a0(1) =                 io.getMotorCoeff(StringUtilities::makeFullyQualifiedRoboDynElement( mechanism,  ApiMap::getXmlElementValue(parametersElement, "A0y")));
        a0(2) =                 io.getMotorCoeff(StringUtilities::makeFullyQualifiedRoboDynElement( mechanism,  ApiMap::getXmlElementValue(parametersElement, "A0z")));

        b(0) =                  io.getMotorCoeff(StringUtilities::makeFullyQualifiedRoboDynElement( mechanism,  ApiMap::getXmlElementValue(parametersElement, "Bx")));
        b(1) =                  io.getMotorCoeff(StringUtilities::makeFullyQualifiedRoboDynElement( mechanism,  ApiMap::getXmlElementValue(parametersElement, "By")));
        b(2) =                  io.getMotorCoeff(StringUtilities::makeFullyQualifiedRoboDynElement( mechanism,  ApiMap::getXmlElementValue(parametersElement, "Bz")));

        c(0) =                  io.getMotorCoeff(StringUtilities::makeFullyQualifiedRoboDynElement( mechanism,  ApiMap::getXmlElementValue(parametersElement, "Cx")));
        c(1) =                  io.getMotorCoeff(StringUtilities::makeFullyQualifiedRoboDynElement( mechanism,  ApiMap::getXmlElementValue(parametersElement, "Cy")));
        c(2) =                  io.getMotorCoeff(StringUtilities::makeFullyQualifiedRoboDynElement( mechanism,  ApiMap::getXmlElementValue(parametersElement, "Cz")));

        d(0) =                  io.getMotorCoeff(StringUtilities::makeFullyQualifiedRoboDynElement( mechanism,  ApiMap::getXmlElementValue(parametersElement, "Dx")));
        d(1) =                  io.getMotorCoeff(StringUtilities::makeFullyQualifiedRoboDynElement( mechanism,  ApiMap::getXmlElementValue(parametersElement, "Dy")));
        d(2) =                  io.getMotorCoeff(StringUtilities::makeFullyQualifiedRoboDynElement( mechanism,  ApiMap::getXmlElementValue(parametersElement, "Dz")));

        d0(0) =                 io.getMotorCoeff(StringUtilities::makeFullyQualifiedRoboDynElement( mechanism,  ApiMap::getXmlElementValue(parametersElement, "D0x")));
        d0(1) =                 io.getMotorCoeff(StringUtilities::makeFullyQualifiedRoboDynElement( mechanism,  ApiMap::getXmlElementValue(parametersElement, "D0y")));
        d0(2) =                 io.getMotorCoeff(StringUtilities::makeFullyQualifiedRoboDynElement( mechanism,  ApiMap::getXmlElementValue(parametersElement, "D0z")));

        pitch(0) =              io.getMotorCoeff(StringUtilities::makeFullyQualifiedRoboDynElement( mechanism,  ApiMap::getXmlElementValue(parametersElement, "Pitchx")));
        pitch(1) =              io.getMotorCoeff(StringUtilities::makeFullyQualifiedRoboDynElement( mechanism,  ApiMap::getXmlElementValue(parametersElement, "Pitchy")));
        pitch(2) =              io.getMotorCoeff(StringUtilities::makeFullyQualifiedRoboDynElement( mechanism,  ApiMap::getXmlElementValue(parametersElement, "Pitchz")));

        yaw(0) =                io.getMotorCoeff(StringUtilities::makeFullyQualifiedRoboDynElement( mechanism,  ApiMap::getXmlElementValue(parametersElement, "Yawx")));
        yaw(1) =                io.getMotorCoeff(StringUtilities::makeFullyQualifiedRoboDynElement( mechanism,  ApiMap::getXmlElementValue(parametersElement, "Yawy")));
        yaw(2) =                io.getMotorCoeff(StringUtilities::makeFullyQualifiedRoboDynElement( mechanism,  ApiMap::getXmlElementValue(parametersElement, "Yawz")));

        m(0) =                  io.getMotorCoeff(StringUtilities::makeFullyQualifiedRoboDynElement( mechanism,  ApiMap::getXmlElementValue(parametersElement, "PitchAxisx")));
        m(1) =                  io.getMotorCoeff(StringUtilities::makeFullyQualifiedRoboDynElement( mechanism,  ApiMap::getXmlElementValue(parametersElement, "PitchAxisy")));
        m(2) =                  io.getMotorCoeff(StringUtilities::makeFullyQualifiedRoboDynElement( mechanism,  ApiMap::getXmlElementValue(parametersElement, "PitchAxisz")));

        n(0) =                  io.getMotorCoeff(StringUtilities::makeFullyQualifiedRoboDynElement( mechanism,  ApiMap::getXmlElementValue(parametersElement, "YawAxisx")));
        n(1) =                  io.getMotorCoeff(StringUtilities::makeFullyQualifiedRoboDynElement( mechanism,  ApiMap::getXmlElementValue(parametersElement, "YawAxisy")));
        n(2) =                  io.getMotorCoeff(StringUtilities::makeFullyQualifiedRoboDynElement( mechanism,  ApiMap::getXmlElementValue(parametersElement, "YawAxisz")));

        float linkLength =      io.getMotorCoeff(StringUtilities::makeFullyQualifiedRoboDynElement( mechanism,  ApiMap::getXmlElementValue(parametersElement, "LinkLength")));

        wrist->loadDesignParams(a,a0,b,c,d,d0,pitch,yaw,m,n,linkLength);
        //wrist->outputDesignParams();

        //! limits
        float upperPitchLim =    io.getMotorCoeff(StringUtilities::makeFullyQualifiedRoboDynElement( mechanism,  ApiMap::getXmlElementValue(parametersElement, "UpperPitchLim")));
        float lowerPitchLim =    io.getMotorCoeff(StringUtilities::makeFullyQualifiedRoboDynElement( mechanism,  ApiMap::getXmlElementValue(parametersElement, "LowerPitchLim")));
        float upperYawLim =      io.getMotorCoeff(StringUtilities::makeFullyQualifiedRoboDynElement( mechanism,  ApiMap::getXmlElementValue(parametersElement, "UpperYawLim")));
        float lowerYawLim =      io.getMotorCoeff(StringUtilities::makeFullyQualifiedRoboDynElement( mechanism,  ApiMap::getXmlElementValue(parametersElement, "LowerYawLim")));
        float upperPitchOffset = io.getMotorCoeff(StringUtilities::makeFullyQualifiedRoboDynElement( mechanism,  ApiMap::getXmlElementValue(parametersElement, "UpperPitchOffset")));
        float upperPitchGain =   io.getMotorCoeff(StringUtilities::makeFullyQualifiedRoboDynElement( mechanism, ApiMap::getXmlElementValue(parametersElement, "UpperPitchGain")));
        float upperYawOffset =   io.getMotorCoeff(StringUtilities::makeFullyQualifiedRoboDynElement( mechanism, ApiMap::getXmlElementValue(parametersElement, "UpperYawOffset")));
        float upperYawGain =     io.getMotorCoeff(StringUtilities::makeFullyQualifiedRoboDynElement( mechanism, ApiMap::getXmlElementValue(parametersElement, "UpperYawGain")));
        float lowerPitchOffset = io.getMotorCoeff(StringUtilities::makeFullyQualifiedRoboDynElement( mechanism, ApiMap::getXmlElementValue(parametersElement, "LowerPitchOffset")));
        float lowerPitchGain =   io.getMotorCoeff(StringUtilities::makeFullyQualifiedRoboDynElement( mechanism, ApiMap::getXmlElementValue(parametersElement, "LowerPitchGain")));
        float lowerYawOffset =   io.getMotorCoeff(StringUtilities::makeFullyQualifiedRoboDynElement( mechanism, ApiMap::getXmlElementValue(parametersElement, "LowerYawOffset")));
        float lowerYawGain =     io.getMotorCoeff(StringUtilities::makeFullyQualifiedRoboDynElement( mechanism, ApiMap::getXmlElementValue(parametersElement, "LowerYawGain")));
        maxSliderDiff =          io.getMotorCoeff(StringUtilities::makeFullyQualifiedRoboDynElement( mechanism,  ApiMap::getXmlElementValue(parametersElement, "MaxSliderDiff")));

        wrist->setLimits(upperPitchLim, lowerPitchLim, upperYawLim, lowerYawLim);
        wrist->setLimitOffsetGain(upperPitchOffset, upperPitchGain, lowerPitchOffset, lowerPitchGain, upperYawOffset, upperYawGain, lowerYawOffset, lowerYawGain);

        //! calibration
        Vector2f sliderOffset, sliderGain, angleOffset, angleGain;
        sliderOffset(0) =       io.getMotorCoeff(StringUtilities::makeFullyQualifiedRoboDynElement( mechanism,  ApiMap::getXmlElementValue(parametersElement, "ThumbSliderOffset")));
        sliderOffset(1) =       io.getMotorCoeff(StringUtilities::makeFullyQualifiedRoboDynElement( mechanism,  ApiMap::getXmlElementValue(parametersElement, "LittleSliderOffset")));

        sliderGain(0) =         io.getMotorCoeff(StringUtilities::makeFullyQualifiedRoboDynElement( mechanism,  ApiMap::getXmlElementValue(parametersElement, "ThumbSliderGain")));
        sliderGain(1) =         io.getMotorCoeff(StringUtilities::makeFullyQualifiedRoboDynElement( mechanism,  ApiMap::getXmlElementValue(parametersElement, "LittleSliderGain")));

        angleOffset(0) =        io.getMotorCoeff(StringUtilities::makeFullyQualifiedRoboDynElement( mechanism,  ApiMap::getXmlElementValue(parametersElement, "PitchOffset")));
        angleOffset(1) =        io.getMotorCoeff(StringUtilities::makeFullyQualifiedRoboDynElement( mechanism,  ApiMap::getXmlElementValue(parametersElement, "YawOffset")));

        angleGain(0) =          io.getMotorCoeff(StringUtilities::makeFullyQualifiedRoboDynElement( mechanism,  ApiMap::getXmlElementValue(parametersElement, "PitchGain")));
        angleGain(1) =          io.getMotorCoeff(StringUtilities::makeFullyQualifiedRoboDynElement( mechanism,  ApiMap::getXmlElementValue(parametersElement, "YawGain")));

        wrist->setSliderOffsetGain(sliderOffset, sliderGain);
        wrist->setAngleOffsetGain(angleOffset, angleGain);

        wrist->maxIt =          io.getMotorCoeff(StringUtilities::makeFullyQualifiedRoboDynElement( mechanism,  ApiMap::getXmlElementValue(parametersElement, "MaxIter")));
        wrist->eps =            io.getMotorCoeff(StringUtilities::makeFullyQualifiedRoboDynElement( mechanism,  ApiMap::getXmlElementValue(parametersElement, "Eps")));

        hta.resize(2);
        float coeff1, coeff2, coeff3, coeff4, scaleFactor;
        coeff1 =                io.getMotorCoeff(StringUtilities::makeFullyQualifiedRoboDynElement( mechanism,   ApiMap::getXmlElementValue(parametersElement, "HallPitchCoeff1")));
        coeff2 =                io.getMotorCoeff(StringUtilities::makeFullyQualifiedRoboDynElement( mechanism,   ApiMap::getXmlElementValue(parametersElement, "HallPitchCoeff2")));
        coeff3 =                io.getMotorCoeff(StringUtilities::makeFullyQualifiedRoboDynElement( mechanism,   ApiMap::getXmlElementValue(parametersElement, "HallPitchCoeff3")));
        coeff4 =                io.getMotorCoeff(StringUtilities::makeFullyQualifiedRoboDynElement( mechanism,   ApiMap::getXmlElementValue(parametersElement, "HallPitchCoeff4")));
        scaleFactor =           io.getMotorCoeff(StringUtilities::makeFullyQualifiedRoboDynElement( mechanism,   ApiMap::getXmlElementValue(parametersElement, "HallScaleFactor")));
        hta[0].useCoeffs(coeff1, coeff2, coeff3, coeff4, scaleFactor);
        coeff1 =                io.getMotorCoeff(StringUtilities::makeFullyQualifiedRoboDynElement( mechanism,   ApiMap::getXmlElementValue(parametersElement, "HallYawCoeff1")));
        coeff2 =                io.getMotorCoeff(StringUtilities::makeFullyQualifiedRoboDynElement( mechanism,   ApiMap::getXmlElementValue(parametersElement, "HallYawCoeff2")));
        coeff3 =                io.getMotorCoeff(StringUtilities::makeFullyQualifiedRoboDynElement( mechanism,   ApiMap::getXmlElementValue(parametersElement, "HallYawCoeff3")));
        coeff4 =                io.getMotorCoeff(StringUtilities::makeFullyQualifiedRoboDynElement( mechanism,   ApiMap::getXmlElementValue(parametersElement, "HallYawCoeff4")));
        hta[1].useCoeffs(coeff1, coeff2, coeff3, coeff4, scaleFactor);

        //! Safety Elements
        maxSensorErrorFault =    io.getMotorCoeff(StringUtilities::makeFullyQualifiedRoboDynElement( mechanism,  ApiMap::getXmlElementValue(parametersElement, "MaxSensorError")));
        maxSliderDiffFault =     io.getMotorCoeff(StringUtilities::makeFullyQualifiedRoboDynElement( mechanism,  ApiMap::getXmlElementValue(parametersElement, "MaxSliderDiffFault")));
        sliderMinPositionFault = io.getMotorCoeff(StringUtilities::makeFullyQualifiedRoboDynElement( mechanism, ApiMap::getXmlElementValue(parametersElement, "SliderMinPositionFault")));
        sliderMaxPositionFault = io.getMotorCoeff(StringUtilities::makeFullyQualifiedRoboDynElement( mechanism, ApiMap::getXmlElementValue(parametersElement, "SliderMaxPositionFault")));
        upperPitchLim =         io.getMotorCoeff(StringUtilities::makeFullyQualifiedRoboDynElement( mechanism,  ApiMap::getXmlElementValue(parametersElement, "UpperPitchFaultLim")));
        lowerPitchLim =         io.getMotorCoeff(StringUtilities::makeFullyQualifiedRoboDynElement( mechanism,  ApiMap::getXmlElementValue(parametersElement, "LowerPitchFaultLim")));
        upperYawLim =           io.getMotorCoeff(StringUtilities::makeFullyQualifiedRoboDynElement( mechanism,  ApiMap::getXmlElementValue(parametersElement, "UpperYawFaultLim")));
        lowerYawLim =           io.getMotorCoeff(StringUtilities::makeFullyQualifiedRoboDynElement( mechanism,  ApiMap::getXmlElementValue(parametersElement, "LowerYawFaultLim")));
        upperPitchOffset =      io.getMotorCoeff(StringUtilities::makeFullyQualifiedRoboDynElement( mechanism,  ApiMap::getXmlElementValue(parametersElement, "UpperPitchFaultOffset")));
        upperPitchGain =        io.getMotorCoeff(StringUtilities::makeFullyQualifiedRoboDynElement( mechanism, ApiMap::getXmlElementValue(parametersElement, "UpperPitchFaultGain")));
        upperYawOffset =        io.getMotorCoeff(StringUtilities::makeFullyQualifiedRoboDynElement( mechanism, ApiMap::getXmlElementValue(parametersElement, "UpperYawFaultOffset")));
        upperYawGain =          io.getMotorCoeff(StringUtilities::makeFullyQualifiedRoboDynElement( mechanism, ApiMap::getXmlElementValue(parametersElement, "UpperYawFaultGain")));
        lowerPitchOffset =      io.getMotorCoeff(StringUtilities::makeFullyQualifiedRoboDynElement( mechanism, ApiMap::getXmlElementValue(parametersElement, "LowerPitchFaultOffset")));
        lowerPitchGain =        io.getMotorCoeff(StringUtilities::makeFullyQualifiedRoboDynElement( mechanism, ApiMap::getXmlElementValue(parametersElement, "LowerPitchFaultGain")));
        lowerYawOffset =        io.getMotorCoeff(StringUtilities::makeFullyQualifiedRoboDynElement( mechanism, ApiMap::getXmlElementValue(parametersElement, "LowerYawFaultOffset")));
        lowerYawGain =          io.getMotorCoeff(StringUtilities::makeFullyQualifiedRoboDynElement( mechanism, ApiMap::getXmlElementValue(parametersElement, "LowerYawFaultGain")));
        wristFault->setLimits(upperPitchLim, lowerPitchLim, upperYawLim, lowerYawLim);
        wristFault->setLimitOffsetGain(upperPitchOffset, upperPitchGain, lowerPitchOffset, lowerPitchGain, upperYawOffset, upperYawGain, lowerYawOffset, lowerYawGain);

        //! Manual Cal elements

        manualCalSliderPos(0) =       io.getMotorCoeff(StringUtilities::makeFullyQualifiedRoboDynElement( mechanism, ApiMap::getXmlElementValue(parametersElement, "ManualCalThumbSlider")));
        manualCalSliderPos(1) =       io.getMotorCoeff(StringUtilities::makeFullyQualifiedRoboDynElement( mechanism, ApiMap::getXmlElementValue(parametersElement, "ManualCalLittleSlider")));
        manualCalAng(0) =       io.getMotorCoeff(StringUtilities::makeFullyQualifiedRoboDynElement( mechanism, ApiMap::getXmlElementValue(parametersElement, "ManualCalPitch")));
        manualCalAng(1) =       io.getMotorCoeff(StringUtilities::makeFullyQualifiedRoboDynElement( mechanism, ApiMap::getXmlElementValue(parametersElement, "ManualCalYaw")));

        desiredQ1 = manualCalSliderPos(0);
        desiredQ2 = manualCalSliderPos(1);
        desiredPitch = manualCalAng(0);
        desiredYaw = manualCalAng(1);
        desiredSlider = Vector2f::Zero();
        desiredAngle = Vector2f::Zero();

        //! live coeff
        //io.setLiveCoeff(nameCalibrationState,      nasa_r2_common_msgs::JointControlCalibrationMode::IGNORE);
        io.setLiveCoeff(namePitchLimit,            0.);
        io.setLiveCoeff(nameYawLimit,              0.);
        io.setLiveCoeff(nameLittlesideSliderLimit, 0.);
        io.setLiveCoeff(nameThumbsideSliderLimit,  0.);
        io.setLiveCoeff(nameSensorError,           0.);
        io.setLiveCoeff(nameSliderDiffError,       0.);
        
        if (!io.hasLiveCoeff(nameUseHalls))
        {
			io.setLiveCoeff(nameUseHalls,              1.);
		}
        wrist->isCalibrated = false;
        calCalled = false;
        calibrationState.state = nasa_r2_common_msgs::JointControlCalibrationMode::IGNORE;
        io.setLiveCoeff(nameCoeffState, nasa_r2_common_msgs::JointControlCoeffState::LOADED);
    }
    else
    {
        stringstream err;
        err << "The file " << parameterFile << " has no element named [ApiMap]";
        RCS::Logger::log("gov.nasa.robonet.JointCommandWrist", Priority::ERROR, err.str());
        throw runtime_error(err.str());
    }
}

void JointCommandWrist::updateMeasuredState(nasa_r2_common_msgs::JointControlData msg)
{
    // If coeffs are not loaded, override with zeros RDEV-1075
    if (msg.coeffState.state != nasa_r2_common_msgs::JointControlCoeffState::LOADED)
    {
        positionVals.assign(completeMessageSize, 0.0);
        velocityVals.assign(completeMessageSize, 0.0);
        return;
    }

    if(!io.hasLiveCoeff(nameCalibrationMode))
    {
        stringstream err;
        err << "Live coeff [" << nameCalibrationMode << "] does not exist.";
        RCS::Logger::log("gov.nasa.robonet.JointCommandWrist", Priority::ERROR, err.str());
        throw runtime_error(err.str());
    }
    if(!io.hasLiveCoeff(nameUseHalls))
    {
        stringstream err;
        err << "Live coeff [" << nameUseHalls << "] does not exist.";
        RCS::Logger::log("gov.nasa.robonet.JointCommandWrist", Priority::ERROR, err.str());
        throw runtime_error(err.str());
    }

    encoder = Vector2f::Zero();
    halls = Vector2f::Zero();
    slider = Vector2f::Zero();
    ang = Vector2f::Zero();

    encoder(0) = io.getInt16(encoderThumbPositionStatusElement);
    encoder(1) = io.getInt16(encoderLittlePositionStatusElement);

    if(io.getLiveCoeff(nameUseHalls)!=0.)
    {
        halls(0) = hta[0].getAngleFromHalls(io.getUInt16(hallsPitchStatusElement));
        halls(1) = hta[1].getAngleFromHalls(io.getUInt16(hallsYawStatusElement));

        // filter halls
        if (!hallFilterInitialized)
        {
            filteredHalls = halls;
            hallFilterInitialized = true;
        }
        filteredHalls = filteredHalls * positionAlpha + halls * (1 - positionAlpha);
    }
    else
    {
        halls(0) = io.getUInt16(hallsPitchStatusElement);
        halls(1) = io.getUInt16(hallsYawStatusElement);
    }

    if(calibrationState.state == nasa_r2_common_msgs::JointControlCalibrationMode::IGNORE)
    {
        //don't do anything
    }
    else if(calibrationState.state == nasa_r2_common_msgs::JointControlCalibrationMode::ENABLE)
    {
        if(io.getLiveCoeff(nameUseHalls) == 0)
        {
            //manual cal
            stringstream ss;
            ss << "Manual calibration on "<<mechanism<<std::endl;
            ss << "slider pos: ("<<manualCalSliderPos(0)<<","<<manualCalSliderPos(1)<<"), angle: ("<<manualCalAng(0)<<","<<manualCalAng(1)<<")";
            RCS::Logger::log("gov.nasa.robonet.JointCommandWrist", Priority::NOTICE, ss.str());
            wrist->calWrist(encoder,  manualCalSliderPos, manualCalAng);

            if(wrist->isCalibrated)
            {
                io.setLiveCoeff(nameCalibrationMode, nasa_r2_common_msgs::JointControlCalibrationMode::DISABLE);
                calibrationState.state = nasa_r2_common_msgs::JointControlCalibrationMode::DISABLE;
                stringstream ss;
                ss << "Calibrated wrist";
                RCS::Logger::log("gov.nasa.robonet.JointCommandWrist", Priority::DEBUG, ss.str());
                //slider = wrist->getSliderFromWristEncoder(encoder);
                //ang = manualCalAng;
                calFailureReported = false;
            }
            else
            {
                if (!calFailureReported)
                {
                    stringstream ss;
                    ss << "Manual calibration failed.. remaining uncalibrated";
                    RCS::Logger::log("gov.nasa.robonet.JointCommandWrist", Priority::WARN, ss.str());
                    calFailureReported = true;
                }
                return;
            }
        }
        else
        {
            //halls cal
            Vector2f calSlider;
            try
            {
                float upperPitch, lowerPitch, upperYaw, lowerYaw;
                wrist->getLimits(halls(0), halls(1), upperPitch, lowerPitch, upperYaw, lowerYaw);
                if(halls(0)>upperPitch || halls(0)<lowerPitch
                        || halls(1)>upperYaw || halls(1)<lowerYaw)
                {
                    stringstream ss;
                    ss<<"Halls data outside of limits";
                    throw std::runtime_error(ss.str());
                }

                calSlider = wrist->getSliderFromAngle(halls);
                stringstream ss;
                ss << "Halls calibration on "<<mechanism<<std::endl;
                ss << "slider pos: ("<<calSlider(0)<<","<<calSlider(1)<<"), angle: ("<<halls(0)<<","<<halls(1)<<")";
                RCS::Logger::log("gov.nasa.robonet.JointCommandWrist", Priority::NOTICE, ss.str());
                wrist->calWrist(encoder, calSlider, halls);
                //! check forward kinematics is good
                wrist->getSliderFromWristEncoder(encoder);
                if(wrist->isCalibrated)
                {
                    io.setLiveCoeff(nameCalibrationMode, nasa_r2_common_msgs::JointControlCalibrationMode::DISABLE);
                    calibrationState.state = nasa_r2_common_msgs::JointControlCalibrationMode::DISABLE;
                    stringstream ss;
                    ss << "Calibrated wrist";
                    RCS::Logger::log("gov.nasa.robonet.JointCommandWrist", Priority::DEBUG, ss.str());
                    calFailureReported = false;
                }
                else
                {
                    if (!calFailureReported)
                    {
                        RCS::Logger::log("gov.nasa.robonet.JointCommandWrist", Priority::WARN, "Calibration failed..  remaining uncalibrated");
                        calFailureReported = true;
                    }
                    return;
                }
            }
            catch(std::exception& e)
            {
                wrist->isCalibrated = false;
                if (!calFailureReported)
                {
                    stringstream ss;
                    ss << "Calibration failed (" << e.what() << "): remaining uncalibrated";
                    RCS::Logger::log("gov.nasa.robonet.JointCommandWrist", Priority::WARN, ss.str());
                    calFailureReported = true;
                }
                return;
            }

        }
    }
    else if (calibrationState.state == nasa_r2_common_msgs::JointControlCalibrationMode::DISABLE)
    {
        if (wrist->isCalibrated)
        {
            slider = wrist->getSliderFromWristEncoder(encoder);

            // filter sliders
            if (!sliderFilterInitialized)
            {
                filteredSlider = slider;
                sliderFilterInitialized = true;
            }
            filteredSlider = filteredSlider * positionAlpha + slider * (1 - positionAlpha);

            try
            {
                ang = wrist->getAngleFromSlider(slider);
                fwdKinFault = false;
            }
            catch(std::runtime_error)
            {
                //! only output this message once until not faulted again.
                if(!fwdKinFault)
                {
                    stringstream ss;
                    ss << "Unable to find fwd kinematics";
                    RCS::Logger::log("gov.nasa.robonet.JointCommandWrist", Priority::WARN, ss.str());
                    fwdKinFault= true;
                }

                if(io.getLiveCoeff(nameUseHalls)!=0)
                {
                    ang = halls;
                }
            }
        }
        setFaultState();

    }
    //std::vector<double> vals(completeMessageSize, 0.);
    //std::vector<double> valDeriv(completeMessageSize, 0.);

    positionVals[0] = ang(0);
    positionVals[1] = ang(1);
    positionVals[2] = slider(0);
    positionVals[3] = slider(1);
    positionVals[4] = halls(0);
    positionVals[5] = halls(1);
    positionVals[6] = encoder(0);
    positionVals[7] = encoder(1);
    positionVals[8] = smoothedPitch;
    positionVals[9] = smoothedYaw;
    positionVals[10] = desiredQ1;
    positionVals[11] = desiredQ2;

    // get velocities
    if (!prevPosInitialized)
    {
        velocityVals.assign(velocityVals.size(), 0.);
        prevPosInitialized = true;
    }
    else
    {
        for (unsigned int i = 0; i < velocityVals.size(); ++i)
        {
            velocityVals[i] = (positionVals[i] - prevPositionVals[i]) / timestep;
        }
    }
    prevPositionVals = positionVals;

    angVel(0) = (float)velocityVals[0];
    angVel(1) = (float)velocityVals[1];
    sliderVel(0) = (float)velocityVals[2];
    sliderVel(1) = (float)velocityVals[3];
    hallsVel(0) = (float)velocityVals[4];
    hallsVel(1) = (float)velocityVals[5];
    encoderVel(0) = (float)velocityVals[6];
    encoderVel(1) = (float)velocityVals[7];
    velocityVals[10] = sliderVelCommand(0);
    velocityVals[11] = sliderVelCommand(1);
        //setFaultState();
    

    return;
}

sensor_msgs::JointState JointCommandWrist::getSimpleMeasuredState()
{
    simpleMeasuredStateMsg.header.stamp = ros::Time::now();

    simpleMeasuredStateMsg.position[0] = positionVals[0];
    simpleMeasuredStateMsg.position[1] = positionVals[1];
    simpleMeasuredStateMsg.position[2] = positionVals[2];
    simpleMeasuredStateMsg.position[3] = positionVals[3];

    simpleMeasuredStateMsg.velocity[0] = velocityVals[0];
    simpleMeasuredStateMsg.velocity[1] = velocityVals[1];
    simpleMeasuredStateMsg.velocity[2] = velocityVals[2];
    simpleMeasuredStateMsg.velocity[3] = velocityVals[3];

    return simpleMeasuredStateMsg;
}

sensor_msgs::JointState JointCommandWrist::getCompleteMeasuredState()
{
    completeMeasuredStateMsg.header.stamp = ros::Time::now();

    completeMeasuredStateMsg.position = positionVals;
    completeMeasuredStateMsg.velocity = velocityVals;

    return completeMeasuredStateMsg;
}

void JointCommandWrist::setFaultState()
{
    //! WristPosLimMonitor
    float up;
    float lp;
    float uy;
    float ly;

    wristFault->getLimits(ang(0), ang(1), up, lp, uy, ly);

    if(ang(0)>up || ang(0)<lp)
    {
        io.setLiveCoeff(namePitchLimit,            1.);
    }
    else
    {
        io.setLiveCoeff(namePitchLimit,            0.);

    }

    if(ang(1)>uy || ang(1)<ly)
    {
        io.setLiveCoeff(nameYawLimit,              1.);
    }
    else
    {
        io.setLiveCoeff(nameYawLimit,              0.);
    }

    //! WristSliderLimMonitor
    if(slider(0) > sliderMaxPositionFault || slider(0) < sliderMinPositionFault )
    {
        std::cout<<"sliderMaxPositionFault: "<<sliderMaxPositionFault<<std::endl;
        std::cout<<"sliderMinPositionFault: "<<sliderMinPositionFault<<std::endl;
        std::cout<<"thumbside slider: "<<slider(0)<<std::endl;
        io.setLiveCoeff(nameLittlesideSliderLimit, 1.);
    }
    else
    {
        io.setLiveCoeff(nameLittlesideSliderLimit, 0.);
    }

    if(slider(1) > sliderMaxPositionFault || slider(1) < sliderMinPositionFault)
    {
        std::cout<<"sliderMaxPositionFault: "<<sliderMaxPositionFault<<std::endl;
        std::cout<<"sliderMinPositionFault: "<<sliderMinPositionFault<<std::endl;
        std::cout<<"thumbside slider: "<<slider(0)<<std::endl;
        io.setLiveCoeff(nameThumbsideSliderLimit,  1.);
    }
    else
    {
        io.setLiveCoeff(nameThumbsideSliderLimit,  0.);
    }

    //! WristSensorErrorMonitor
    if(!io.hasLiveCoeff(nameUseHalls))
    {
        stringstream err;
        err << "Live coeff [" << nameUseHalls << "] does not exist.";
        RCS::Logger::log("gov.nasa.robonet.JointCommandWrist", Priority::ERROR, err.str());
        throw runtime_error(err.str());
    }
    if(io.getLiveCoeff(nameUseHalls) != 0.)
    {
        if(fabs(ang(0)-halls(0)) > maxSensorErrorFault || fabs(ang(1)-halls(1)) > maxSensorErrorFault)
        {
            io.setLiveCoeff(nameSensorError,           1.);
        }
        else
        {
            io.setLiveCoeff(nameSensorError,           0.);
        }
    }

    //! WristSliderDiffMonitor
    if(fabs(slider(0)-slider(1))> maxSliderDiffFault)
    {
        io.setLiveCoeff(nameSliderDiffError,       1.);
    }
    else
    {
        io.setLiveCoeff(nameSliderDiffError,       0.);
    }

}

nasa_r2_common_msgs::JointCommand JointCommandWrist::getCommandedState()
{
    commandedStateMsg.header.stamp = ros::Time::now();
    return commandedStateMsg;
}

void JointCommandWrist::setCommand(nasa_r2_common_msgs::JointCommand msg, nasa_r2_common_msgs::JointControlData control)
{
    if(!calCalled && control.calibrationMode.state == nasa_r2_common_msgs::JointControlCalibrationMode::ENABLE)
    {
        calibrationState.state = nasa_r2_common_msgs::JointControlCalibrationMode::ENABLE;
        calCalled=true;
    }
    if(control.calibrationMode.state == nasa_r2_common_msgs::JointControlCalibrationMode::DISABLE)
    {
        calCalled=false;
    }

//    commandedStateMsg.name = roboDynJoints;

    //! confirm msg has required parts
    if (!msg.name.empty() && !msg.desiredPosition.empty())
    {
        if (msg.desiredPositionVelocityLimit.empty())
            msg.desiredPositionVelocityLimit.resize(msg.name.size(), 0.);

        //! get the desiredQ1 and desiredQ2 from the current ControlCommandMode we are in
        if (control.commandMode.state == nasa_r2_common_msgs::JointControlCommandMode::MOTCOM)
        {
            for (unsigned int i = 0; i < msg.name.size(); ++i)
            {
                if (msg.name[i] == roboDynActuators[0])
                    desiredQ1 = msg.desiredPosition[i];
                else if (msg.name[i] == roboDynActuators[1])
                    desiredQ2 = msg.desiredPosition[i];
            }
        }
        else if (control.commandMode.state == nasa_r2_common_msgs::JointControlCommandMode::ACTUATOR)
        {
            for (unsigned int i = 0; i < msg.name.size(); ++i)
            {
                if (msg.name[i] == roboDynActuators[0])
                    desiredQ1 = msg.desiredPosition[i];
                else if (msg.name[i] == roboDynActuators[1])
                    desiredQ2 = msg.desiredPosition[i];
            }
            try
            {
                limitSliderDiff(desiredQ1, desiredQ2);
                desiredAngle = wrist->getAngleFromSlider(Vector2f(desiredQ1, desiredQ2));
                wrist->applyLimits(desiredAngle(0), desiredAngle(1));

                // smooth if in drive and not in actuator
                smoothedPitch = smootherPitch->reset(desiredAngle(0), 0.);
                smoothedYaw = smootherYaw->reset(desiredAngle(1), 0.);

                commandedStateMsg.desiredPosition[0] = desiredAngle(0);
                commandedStateMsg.desiredPosition[1] = desiredAngle(1);
                commandedStateMsg.desiredPositionVelocityLimit[0] = 0;
                commandedStateMsg.desiredPositionVelocityLimit[1] = 0;
            }
            catch(std::runtime_error &e)
            {
                //! if we've errored out, there is something wrong with the kinematics here, send to park
                RCS::Logger::getCategory("gov.nasa.robonet.JointCommandWrist")<<log4cpp::Priority::ERROR<<"Kinematics failed with error "<<e.what()<<" on ("<<desiredQ1<<","<<desiredQ2<<" command to actuators " << roboDynActuators[0]<<", "<< roboDynActuators[1];
                control.controlMode.state = nasa_r2_common_msgs::JointControlMode::PARK;
            }

        }
        else
        {
            if (msg.desiredPositionVelocityLimit.empty())
            {
                msg.desiredPositionVelocityLimit.resize(msg.name.size(), 0);
            }

            for (unsigned int i = 0; i < msg.name.size(); ++i)
            {
                if(msg.name[i] == roboDynJoints[0])
                {
                    desiredPitch = msg.desiredPosition[i];
                    desiredPitchVel = msg.desiredPositionVelocityLimit[i];
                    //RCS::Logger::getCategory("gov.nasa.robonet.JointCommandWrist")<<log4cpp::Priority::DEBUG<<"received desired pitch " << commandedStateMsg.desiredPosition[0];
                }
                else if (msg.name[i] == roboDynJoints[1])
                {
                    desiredYaw = msg.desiredPosition[i];
                    desiredYawVel = msg.desiredPositionVelocityLimit[i];
                    //RCS::Logger::getCategory("gov.nasa.robonet.JointCommandWrist")<<log4cpp::Priority::DEBUG<<"received desired yaw " << commandedStateMsg.desiredPosition[1];
                }
            }
            try
            {
                if(control.controlMode.state == nasa_r2_common_msgs::JointControlMode::DRIVE)
                {
                    // smooth in drive
                    smoothedPitch = smootherPitch->update(desiredPitch, desiredPitchVel);
                    smoothedYaw = smootherYaw->update(desiredYaw, desiredYawVel);
                }
                else
                {
                    smoothedPitch = smootherPitch->reset(desiredPitch, desiredPitchVel);
                    smoothedYaw = smootherYaw->reset(desiredYaw, desiredYawVel);
                }
                wrist->applyLimits(smoothedPitch, smoothedYaw);
                desiredSlider = wrist->getSliderFromAngle(Vector2f(smoothedPitch, smoothedYaw));
                limitSliderDiff(desiredSlider(0), desiredSlider(1));
                desiredQ1 = desiredSlider(0);
                desiredQ2 = desiredSlider(1);
                commandedStateMsg.desiredPosition[0] = desiredPitch;
                commandedStateMsg.desiredPosition[1] = desiredYaw;
                commandedStateMsg.desiredPositionVelocityLimit[0] = desiredPitchVel;
                commandedStateMsg.desiredPositionVelocityLimit[1] = desiredYawVel;
            }
            catch(std::runtime_error)
            {
                //! if we've errored out, there is something wrong with the kinematics here, send to park
                RCS::Logger::getCategory("gov.nasa.robonet.JointCommandWrist")<<log4cpp::Priority::ERROR<<"Kinematics failed on ("<<desiredPitch<<","<<desiredYaw<<" command to joints " << roboDynJoints[0]<<", "<< roboDynJoints[1];
                control.controlMode.state = nasa_r2_common_msgs::JointControlMode::PARK;
            }
        }
    }

    //! set the motcoms
    if(control.controlMode.state == nasa_r2_common_msgs::JointControlMode::DRIVE && wrist->isCalibrated)
    {
        double motcom1 = q1Controller->update(desiredQ1, (double)slider(0), (double)sliderVel(0), busVoltage, sliderVelCommand(0));
        double motcom2 = q2Controller->update(desiredQ2, (double)slider(1), (double)sliderVel(1), busVoltage, sliderVelCommand(1));

        io.setInt16(desiredMotComThumbCommandElement,             motcom1);
        io.setInt16(desiredMotComLittleCommandElement,            motcom2);
    }
    else
    {
        //! we are not in drive, so set motcoms to 0
        io.setInt16(desiredMotComThumbCommandElement, 0);
        io.setInt16(desiredMotComLittleCommandElement, 0);
    }
        

}

void JointCommandWrist::limitSliderDiff(float &slider1, float &slider2)
{
    //! limit slider diff
    if(fabs(slider1-slider2) > maxSliderDiff)
    {
        stringstream err;
        err << "setCommand() requires the difference in sliders to be below " << maxSliderDiff<<" in JointCommand.* when setCommand() is called... adjusting sliders.";
        RCS::Logger::log("gov.nasa.robonet.JointCommandWrist", Priority::INFO, err.str());

        if(fabs(slider1-manualCalSliderPos(0))>fabs(slider2-manualCalSliderPos(1)))
        {
            //Q1 is further away from the cal point than q2, bring q1 in to q2+maxSliderDiff
            if(slider1>slider2)
            {
                RCS::Logger::getCategory("gov.nasa.robonet.JointCommandWrist")<<log4cpp::Priority::DEBUG<<"limiting slider diff: (" << slider1<<","<<slider2<<") to ("<<slider2 + maxSliderDiff<<","<< slider2<<")";
                slider1 = slider2 + maxSliderDiff;
            }
            else
            {
                RCS::Logger::getCategory("gov.nasa.robonet.JointCommandWrist")<<log4cpp::Priority::DEBUG<<"limiting slider diff: (" << slider1<<","<<slider2<<") to ("<<slider2 - maxSliderDiff<<","<< slider2<<")";
                slider1 = slider2 - maxSliderDiff;
            }
        }
        else
        {
            //Q2 is further away
            if(slider2 > slider1)
            {
                RCS::Logger::getCategory("gov.nasa.robonet.JointCommandWrist")<<log4cpp::Priority::DEBUG<<"limiting slider diff: (" << slider1<<","<<slider2<<") to ("<<slider1<<","<< slider1 + maxSliderDiff<<")";
                slider2 = slider1 + maxSliderDiff;
            }
            else
            {
                RCS::Logger::getCategory("gov.nasa.robonet.JointCommandWrist")<<log4cpp::Priority::DEBUG<<"limiting slider diff: (" << slider1<<","<<slider2<<") to ("<<slider1 <<","<< slider1 - maxSliderDiff<<")";
                slider2 = slider1 - maxSliderDiff;
            }
        }
    }

}

nasa_r2_common_msgs::JointCapability JointCommandWrist::getCapability()
{
    float up, uy, lp, ly;
    wrist->getLimits(ang(0), ang(1), up, lp, uy, ly);
    jointCapabilityMsg.positionLimitMax[0] = up;
    jointCapabilityMsg.positionLimitMax[1] = uy;
    jointCapabilityMsg.positionLimitMin[0] = lp;
    jointCapabilityMsg.positionLimitMin[1] = ly;

    return jointCapabilityMsg;
}
