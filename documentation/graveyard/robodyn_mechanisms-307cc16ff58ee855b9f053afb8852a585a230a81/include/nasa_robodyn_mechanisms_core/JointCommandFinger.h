/*!
 * @file  JointCommandFinger.h
 * @brief Defines the JointCommandFinger  class.
 * @author Ross C. Taylor
 * @date 2013-03-05
 */

#ifndef JOINTCOMMANDFINGER_H
#define JOINTCOMMANDFINGER_H

#include "nasa_robodyn_mechanisms_core/JointCommandInterface.h"
#include "nasa_r2_config_core/StringUtilities.h"
#include "diagnostic_msgs/DiagnosticArray.h"
#include "nasa_robodyn_mechanisms_core/FingerController.h"
#include "EmbeddedSmoother.h"
#include <boost/ptr_container/ptr_array.hpp>

/***************************************************************************//**
 *
 * @brief This class provides the joint command interface for the fingers
 *
 ******************************************************************************/

template <unsigned int N>
class JointCommandFinger : public JointCommandInterface
{
public:
    JointCommandFinger(const std::string& mechanism, IoFunctions ioFunctions);
    ~JointCommandFinger();

    sensor_msgs::JointState           getSimpleMeasuredState();
    sensor_msgs::JointState           getCompleteMeasuredState();
    nasa_r2_common_msgs::JointCommand getCommandedState();
    void setCommand(nasa_r2_common_msgs::JointCommand msg, nasa_r2_common_msgs::JointControlData control);
    void updateMeasuredState(nasa_r2_common_msgs::JointControlData msg);
    void setFaultState();

    nasa_r2_common_msgs::JointCapability getCapability();
    void loadCoeffs();

private:
    FingerController<N> controller;
    boost::array<std::string, N>            hallStateElements;
    boost::array<std::string, N+1>          encoderStateElements;
    boost::array<std::string, N+1>          tensionAStateElements;
    boost::array<std::string, N+1>          tensionBStateElements;
    boost::array<std::string, N+1>          motComCommandElements;

    typename FingerController<N>::JointVectorType    jointVec;
    typename FingerController<N>::JointVectorType    hallVec;
    typename FingerController<N>::SliderVectorType   encoderVec, encoderOffset, sliderVec, desiredSliders, desiredSliderVels;
    typename FingerController<N>::SliderVectorType   tensionAVec, tensionBVec, tensionVec;

    bool tubeTared;
    double ktendon;
    bool activeStiffnessOn;
    bool useCartesian;
    bool isLeft;
    double tensionMin;

    std::vector<double> positionVals;
    std::vector<double> velocityVals;
    std::vector<double> effortVals;

    // util
    std::vector<std::string>::iterator strVecIt;
    boost::ptr_array<EmbeddedSmoother, N> smoother;
    typename FingerController<N>::JointVectorType smoothedPos;
    typename FingerController<N>::JointVectorType prevSmoothedPos;

    // filter values
    typename FingerController<N>::SliderVectorType filteredSliderVec;
    typename FingerController<N>::JointVectorType  filteredHallVec;
    std::vector<double> prevPositionVals;
    double timestep;
    double positionAlpha;
    bool filterInitialized;

    // live coeff names
    std::string nameIsCalibrated;
    std::string kTendonName;
    boost::array<std::string, N+1>          encoderOffsetElements;
    boost::array<std::string, N+1>          sliderOffsetElements;
    boost::array<std::string, N+1>          sliderTarePosElements;
    boost::array<std::string, N+1>          tensionOffsetElements;
};

template <unsigned int N>
JointCommandFinger<N>::JointCommandFinger(const std::string& mechanism, IoFunctions ioFunctions)
    : JointCommandInterface(mechanism, ioFunctions)
    , desiredSliders(FingerController<N>::SliderVectorType::Zero())
    , tubeTared(false), ktendon(1.0) , activeStiffnessOn(false), useCartesian(false), isLeft(false), tensionMin(0.0), filterInitialized(false)
{
    if (mechanism == "")
    {
        std::stringstream err;
        err << "Constructor requires mechanism be non-empty.";
        RCS::Logger::log("gov.nasa.robonet.JointCommandFinger", log4cpp::Priority::FATAL, err.str());
        throw std::invalid_argument(err.str());
    }

    if (io.getUInt16.empty())
    {
        std::stringstream err;
        err << "Constructor requires 'io.getUInt16' be non-empty.";
        RCS::Logger::log("gov.nasa.robonet.JointCommandFinger", log4cpp::Priority::FATAL, err.str());
        throw std::invalid_argument(err.str());
    }
    if (io.getInt16.empty())
    {
        std::stringstream err;
        err << "Constructor requires 'io.getInt16' be non-empty.";
        RCS::Logger::log("gov.nasa.robonet.JointCommandFinger", log4cpp::Priority::FATAL, err.str());
        throw std::invalid_argument(err.str());
    }
    if (io.setInt16.empty())
    {
        std::stringstream err;
        err << "Constructor requires 'io.setInt16' be non-empty.";
        RCS::Logger::log("gov.nasa.robonet.JointCommandFinger", log4cpp::Priority::FATAL, err.str());
        throw std::invalid_argument(err.str());
    }
    if (io.getBrainstemCoeff.empty())
    {
        std::stringstream err;
        err << "Constructor requires 'io.getBrainstemCoeff' be non-empty.";
        RCS::Logger::log("gov.nasa.robonet.JointCommandFinger", log4cpp::Priority::FATAL, err.str());
        throw std::invalid_argument(err.str());
    }
    if (io.getMotorCoeff.empty())
    {
        std::stringstream err;
        err << "Constructor requires 'io.getMotorCoeff' be non-empty.";
        RCS::Logger::log("gov.nasa.robonet.JointCommandFinger", log4cpp::Priority::FATAL, err.str());
        throw std::invalid_argument(err.str());
    }
    if (io.hasLiveCoeff.empty())
    {
        std::stringstream err;
        err << "Constructor requires 'io.hasLiveCoeff' be non-empty.";
        RCS::Logger::log("gov.nasa.robonet.JointCommandFinger", log4cpp::Priority::FATAL, err.str());
        throw std::invalid_argument(err.str());
    }
    if (io.getLiveCoeff.empty())
    {
        std::stringstream err;
        err << "Constructor requires 'io.getLiveCoeff' be non-empty.";
        RCS::Logger::log("gov.nasa.robonet.JointCommandFinger", log4cpp::Priority::FATAL, err.str());
        throw std::invalid_argument(err.str());
    }
    if (io.setLiveCoeff.empty())
    {
        std::stringstream err;
        err << "Constructor requires 'io.setLiveCoeff' be non-empty.";
        RCS::Logger::log("gov.nasa.robonet.JointCommandFinger", log4cpp::Priority::FATAL, err.str());
        throw std::invalid_argument(err.str());
    }
    if (io.getJointNames.empty())
    {
        std::stringstream err;
        err << "Constructor requires 'io.getJointNames' be non-empty.";
        RCS::Logger::log("gov.nasa.robonet.JointCommandFinger", log4cpp::Priority::FATAL, err.str());
        throw std::invalid_argument(err.str());
    }
    if (io.getActuatorNames.empty())
    {
        std::stringstream err;
        err << "Constructor requires 'io.getActuatorNames' be non-empty.";
        RCS::Logger::log("gov.nasa.robonet.JointCommandFinger", log4cpp::Priority::FATAL, err.str());
        throw std::invalid_argument(err.str());
    }
    if (io.getCommandFile.empty())
    {
        std::stringstream err;
        err << "Constructor requires 'io.getCommandFile' be non-empty.";
        RCS::Logger::log("gov.nasa.robonet.JointCommandFinger", log4cpp::Priority::FATAL, err.str());
        throw std::invalid_argument(err.str());
    }

    roboDynJoints    = io.getJointNames(mechanism);
    roboDynActuators = io.getActuatorNames(mechanism);

    if(roboDynJoints.size() != N or
       roboDynActuators.size() != N+1)
    {
        std::stringstream err;
        err << "Constructor requires roboDynJoints have N and roboDynActuators have N+1 values, with N for this instance being " << N <<std::endl;
        err << "The current size of roboDynJoints: " << roboDynJoints.size() << std::endl;
        err << "The current size of roboDynActuators: " << roboDynActuators.size() << std::endl;
        RCS::Logger::log("gov.nasa.robonet.JointCommandFinger", log4cpp::Priority::FATAL, err.str());
        throw std::invalid_argument(err.str());

    }

    loadCoeffs();
}

template <unsigned int N>
JointCommandFinger<N>::~JointCommandFinger()
{}

template <unsigned int N>
void JointCommandFinger<N>::loadCoeffs()
{
    tubeTared = false;
    ktendon = 1.0;
    activeStiffnessOn = false;
    useCartesian = false;
    isLeft = false;
    tensionMin = 0.0;
    filterInitialized = false;

    std::string parameterFile = io.getCommandFile(mechanism);

    //! Initialize messages
    unsigned int numSimpleStates = 2 * N + 1; // n joints, n+1 sliders (with tension/effort)
    simpleMeasuredStateMsg.name.resize(numSimpleStates);
    simpleMeasuredStateMsg.position.resize(numSimpleStates, 0.);
    simpleMeasuredStateMsg.velocity.resize(numSimpleStates, 0.);
    simpleMeasuredStateMsg.effort.resize(numSimpleStates, 0.);

    unsigned int numCompleteStates = numSimpleStates + 2 * N + 1; // joint and slider embedded commands
    completeMeasuredStateMsg.name.resize(numCompleteStates);
    completeMeasuredStateMsg.position.resize(numCompleteStates, 0.);
    completeMeasuredStateMsg.velocity.resize(numCompleteStates, 0.);
    completeMeasuredStateMsg.effort.resize(numCompleteStates, 0.);

    commandedStateMsg.name = roboDynJoints;
    commandedStateMsg.desiredPosition.resize(N, 0.);
    commandedStateMsg.desiredPositionVelocityLimit.resize(N, 0.);
    commandedStateMsg.feedForwardTorque.resize(N, 0.);
    commandedStateMsg.proportionalGain.resize(N, 0.);
    commandedStateMsg.derivativeGain.resize(N, 0.);
    commandedStateMsg.integralGain.resize(N, 0.);
    commandedStateMsg.positionLoopTorqueLimit.resize(N, 0.);
    commandedStateMsg.positionLoopWindupLimit.resize(N, 0.);
    commandedStateMsg.torqueLoopVelocityLimit.resize(N, 0.);

    jointCapabilityMsg.name = roboDynJoints;
    jointCapabilityMsg.positionLimitMax.resize(N, 0.);
    jointCapabilityMsg.positionLimitMin.resize(N, 0.);
    jointCapabilityMsg.torqueLimit.resize(N, 0.);

    for (unsigned int i = 0; i < N; ++i)
    {
        simpleMeasuredStateMsg.name[i] = roboDynJoints[i];
    }
    for (unsigned int i = 0; i < N+1; ++i)
    {
        simpleMeasuredStateMsg.name[N+i] = roboDynActuators[i];
    }

    for (unsigned int i = 0; i < N; ++i)
    {
        completeMeasuredStateMsg.name[i] = roboDynJoints[i];
        completeMeasuredStateMsg.name[2*N+1+i] = roboDynJoints[i] + "/embeddedCommand";
    }
    for (unsigned int i = 0; i < N+1; ++i)
    {
        completeMeasuredStateMsg.name[N+i] = roboDynActuators[i];
        completeMeasuredStateMsg.name[3*N+1+i] = roboDynActuators[i] + "/embeddedCommand";
    }

    positionVals.resize(numSimpleStates, 0.);
    velocityVals.resize(numSimpleStates, 0.);
    effortVals.resize(numSimpleStates, 0.);
    prevPositionVals = positionVals;

    //! Parse parameter file
    TiXmlDocument file(parameterFile.c_str());
    bool loadOkay = file.LoadFile();
    if (!loadOkay)
    {
        std::stringstream err;
        err << "Failed to load file [" << parameterFile << "]";
        RCS::Logger::log("gov.nasa.robonet.JointCommandFinger", log4cpp::Priority::FATAL, err.str());
        throw std::runtime_error(err.str());
    }
    TiXmlHandle doc(&file);
    RCS::Logger::log("gov.nasa.robonet.JointCommandFinger", log4cpp::Priority::INFO, "CommandFile [" + parameterFile + "] successfully loaded.");

    // Check for ApiMap
    TiXmlHandle parametersElement(doc.FirstChildElement("ApiMap"));

    if (parametersElement.ToElement())
    {
        //! State & Command Elements
        std::stringstream str;
        for (unsigned int i = 0; i < N+1; ++i)
        {
            str.str("");
            str << "Actuator" << i << "EncoderState";
            encoderStateElements[i] = StringUtilities::makeFullyQualifiedRoboDynElement(roboDynActuators[i], ApiMap::getXmlElementValue(parametersElement, str.str()));
            str.str("");
            str << "Actuator" << i << "TensionAState";
            tensionAStateElements[i] = StringUtilities::makeFullyQualifiedRoboDynElement(roboDynActuators[i], ApiMap::getXmlElementValue(parametersElement, str.str()));
            str.str("");
            str << "Actuator" << i << "TensionBState";
            tensionBStateElements[i] = StringUtilities::makeFullyQualifiedRoboDynElement(roboDynActuators[i], ApiMap::getXmlElementValue(parametersElement, str.str()));
            str.str("");
            str << "Actuator" << i << "MotComCommand";
            motComCommandElements[i] = StringUtilities::makeFullyQualifiedRoboDynElement(roboDynActuators[i], ApiMap::getXmlElementValue(parametersElement, str.str()));
        }

        for (unsigned int i = 0; i < N; ++i)
        {
            str.str("");
            str << "Joint" << i << "HallState";
            hallStateElements[i] = StringUtilities::makeFullyQualifiedRoboDynElement(roboDynJoints[i], ApiMap::getXmlElementValue(parametersElement, str.str()));
        }

        //! Coeffs
        // finger controller coeffs
        float tempVal;
        tempVal = io.getMotorCoeff(StringUtilities::makeFullyQualifiedRoboDynElement( mechanism,  ApiMap::getXmlElementValue(parametersElement, "MillimetersPerCount")));
        controller.setMillimetersPerCount(static_cast<double>(tempVal));

        double hallScale;
        typename FingerController<N>::JointVectorType hallCoeffs0, hallCoeffs1, hallCoeffs2, hallCoeffs3;
        hallScale = io.getMotorCoeff(StringUtilities::makeFullyQualifiedRoboDynElement( mechanism,  ApiMap::getXmlElementValue(parametersElement, "HallScaleFactor")));
        for (unsigned int i = 0; i < hallCoeffs0.rows(); ++i)
        {
            str.str("");
            str << "Joint" << i << "HallCoeff0";
            hallCoeffs0[i] = io.getMotorCoeff(StringUtilities::makeFullyQualifiedRoboDynElement( mechanism,  ApiMap::getXmlElementValue(parametersElement, str.str())));
            str.str("");
            str << "Joint" << i << "HallCoeff1";
            hallCoeffs1[i] = io.getMotorCoeff(StringUtilities::makeFullyQualifiedRoboDynElement( mechanism,  ApiMap::getXmlElementValue(parametersElement, str.str())));
            str.str("");
            str << "Joint" << i << "HallCoeff2";
            hallCoeffs2[i] = io.getMotorCoeff(StringUtilities::makeFullyQualifiedRoboDynElement( mechanism,  ApiMap::getXmlElementValue(parametersElement, str.str())));
            str.str("");
            str << "Joint" << i << "HallCoeff3";
            hallCoeffs3[i] = io.getMotorCoeff(StringUtilities::makeFullyQualifiedRoboDynElement( mechanism,  ApiMap::getXmlElementValue(parametersElement, str.str())));
        }
        controller.setHallAngleParameters(hallScale, hallCoeffs0, hallCoeffs1, hallCoeffs2, hallCoeffs3);

        typename FingerController<N>::SliderVectorType tensionParam1, tensionParam2, tensionParam3, tensionParam4;
        for (unsigned int i = 0; i < tensionParam1.rows(); ++i)
        {
            str.str("");
            str << "Actuator" << i << "TensionAGain";
            tensionParam1[i] = io.getMotorCoeff(StringUtilities::makeFullyQualifiedRoboDynElement( mechanism,  ApiMap::getXmlElementValue(parametersElement, str.str())));
            str.str("");
            str << "Actuator" << i << "TensionBGain";
            tensionParam2[i] = io.getMotorCoeff(StringUtilities::makeFullyQualifiedRoboDynElement( mechanism,  ApiMap::getXmlElementValue(parametersElement, str.str())));
            str.str("");
            str << "Actuator" << i << "TensionSensorCalOffset";
            tensionParam3[i] = io.getMotorCoeff(StringUtilities::makeFullyQualifiedRoboDynElement( mechanism,  ApiMap::getXmlElementValue(parametersElement, str.str())));
            str.str("");
            str << "Actuator" << i << "CalibratedStrain";
            tensionParam4[i] = io.getMotorCoeff(StringUtilities::makeFullyQualifiedRoboDynElement( mechanism,  ApiMap::getXmlElementValue(parametersElement, str.str())));
        }
        controller.setTensionParameters(tensionParam1, tensionParam2, tensionParam3, tensionParam4);
        
        tempVal = io.getMotorCoeff(StringUtilities::makeFullyQualifiedRoboDynElement( mechanism,  ApiMap::getXmlElementValue(parametersElement, "ActiveStiffnessOn")));
        activeStiffnessOn = (tempVal <= 0 ? false : true);
        tempVal = io.getMotorCoeff(StringUtilities::makeFullyQualifiedRoboDynElement( mechanism,  ApiMap::getXmlElementValue(parametersElement, "UseCartesian")));
        useCartesian = (tempVal <= 0 ? false : true);
        isLeft = (mechanism.find("left") > 10 ? false : true);

        double kd, tensionMax, tensionKp, tFreq, deadband;
        typename FingerController<N>::JointVectorType stiffnessK, jointKp;
        kd = io.getMotorCoeff(StringUtilities::makeFullyQualifiedRoboDynElement( mechanism,  ApiMap::getXmlElementValue(parametersElement, "KdGain")));
        tensionMin = io.getMotorCoeff(StringUtilities::makeFullyQualifiedRoboDynElement( mechanism,  ApiMap::getXmlElementValue(parametersElement, "MinTension")));
        tensionMax = io.getMotorCoeff(StringUtilities::makeFullyQualifiedRoboDynElement( mechanism,  ApiMap::getXmlElementValue(parametersElement, "MaxTension")));
        tensionKp = io.getMotorCoeff(StringUtilities::makeFullyQualifiedRoboDynElement( mechanism,  ApiMap::getXmlElementValue(parametersElement, "tensionKpGain")));
        tFreq = io.getMotorCoeff(StringUtilities::makeFullyQualifiedRoboDynElement( mechanism,  ApiMap::getXmlElementValue(parametersElement, "tensionFilterCutoffFreq")));
        deadband = io.getMotorCoeff(StringUtilities::makeFullyQualifiedRoboDynElement( mechanism,  ApiMap::getXmlElementValue(parametersElement, "DeadbandSize")));
        for (unsigned int i = 0; i < stiffnessK.rows(); ++i)
        {
            str.str("");
            str << "Joint" << i << "stiffnessKGain";
            stiffnessK[i] = io.getMotorCoeff(StringUtilities::makeFullyQualifiedRoboDynElement( mechanism,  ApiMap::getXmlElementValue(parametersElement, str.str())));
            str.str("");
            str << "Joint" << i << "torqueKpGain";
            jointKp[i] = io.getMotorCoeff(StringUtilities::makeFullyQualifiedRoboDynElement( mechanism,  ApiMap::getXmlElementValue(parametersElement, str.str())));
        }
        controller.setStiffnessGains(stiffnessK, kd, tensionMin, tensionMax, jointKp, tensionKp, tFreq, deadband);

        tempVal = io.getBrainstemCoeff(StringUtilities::makeFullyQualifiedRoboDynElement( mechanism,  ApiMap::getXmlElementValue(parametersElement, "BusVoltage")));
        controller.setBusVoltage(static_cast<double>(tempVal));

        // filter
        positionAlpha = io.getMotorCoeff(StringUtilities::makeFullyQualifiedRoboDynElement( mechanism,  ApiMap::getXmlElementValue(parametersElement, "PositionAlpha")));

        // "embedded" smoothing
        EmbeddedSmoother::Settings smoothSettings;
        timestep = io.getMotorCoeff(StringUtilities::makeFullyQualifiedRoboDynElement( mechanism,  ApiMap::getXmlElementValue(parametersElement, "Timestep")));
        if (timestep <= 0.)
        {
            throw std::runtime_error("invalid timestep");
        }
        smoothSettings.timestep = timestep;
        for (unsigned int i = 0; i < N; ++i)
        {
            str.str("");
            str << "Joint" << i << "MinimumVelocity";
            smoothSettings.minVel = io.getMotorCoeff(StringUtilities::makeFullyQualifiedRoboDynElement( mechanism,  ApiMap::getXmlElementValue(parametersElement, str.str())));
            str.str("");
            str << "Joint" << i << "MaximumAcceleration";
            smoothSettings.maxAcc = io.getMotorCoeff(StringUtilities::makeFullyQualifiedRoboDynElement( mechanism,  ApiMap::getXmlElementValue(parametersElement, str.str())));
            str.str("");
            str << "Joint" << i << "AccelerationGain";
            smoothSettings.accGain = io.getMotorCoeff(StringUtilities::makeFullyQualifiedRoboDynElement( mechanism,  ApiMap::getXmlElementValue(parametersElement, str.str())));
            smoother.replace(i, new EmbeddedSmoother(smoothSettings));
        }

        /// multiloop parameters
        MultiLoopController tempController;
        tempController.setLoopRate(1./timestep);

        double bypass =             io.getMotorCoeff(StringUtilities::makeFullyQualifiedRoboDynElement( mechanism,  ApiMap::getXmlElementValue(parametersElement, "PositionLoopBypass")));
        double sliderMinPosition =  io.getMotorCoeff(StringUtilities::makeFullyQualifiedRoboDynElement( mechanism,  ApiMap::getXmlElementValue(parametersElement, "SliderMinPositionCommand")));
        double sliderMaxPosition =  io.getMotorCoeff(StringUtilities::makeFullyQualifiedRoboDynElement( mechanism,  ApiMap::getXmlElementValue(parametersElement, "SliderMaxPositionCommand")));
        double pKp               =  io.getMotorCoeff(StringUtilities::makeFullyQualifiedRoboDynElement( mechanism,  ApiMap::getXmlElementValue(parametersElement, "SliderKpGainCommand")));

        tempController.setPositionLoopParameters(sliderMinPosition, sliderMaxPosition, pKp, bypass <= 0 ? false : true);
        
        bypass =               io.getMotorCoeff(StringUtilities::makeFullyQualifiedRoboDynElement( mechanism,  ApiMap::getXmlElementValue(parametersElement, "TorqueLoopBypass")));
        double minTension =    io.getMotorCoeff(StringUtilities::makeFullyQualifiedRoboDynElement( mechanism,  ApiMap::getXmlElementValue(parametersElement, "SliderMinTensionCommand")));
        double maxTension =    io.getMotorCoeff(StringUtilities::makeFullyQualifiedRoboDynElement( mechanism,  ApiMap::getXmlElementValue(parametersElement, "SliderMaxTensionCommand")));
        double tKp        =    io.getMotorCoeff(StringUtilities::makeFullyQualifiedRoboDynElement( mechanism,  ApiMap::getXmlElementValue(parametersElement, "SliderTensionKpGainCommand")));
        double offset     =    io.getMotorCoeff(StringUtilities::makeFullyQualifiedRoboDynElement( mechanism,  ApiMap::getXmlElementValue(parametersElement, "SliderTensionOffsetCommand")));

        tempController.setTorqueLoopParameters(minTension, maxTension, tKp, offset, bypass <= 0 ? false : true);

        bypass =                io.getMotorCoeff(StringUtilities::makeFullyQualifiedRoboDynElement( mechanism,  ApiMap::getXmlElementValue(parametersElement, "VelocityLoopBypass")));
        double minVelocity =    io.getMotorCoeff(StringUtilities::makeFullyQualifiedRoboDynElement( mechanism,  ApiMap::getXmlElementValue(parametersElement, "SliderMinVelocityCommand" )));
        double maxVelocity =    io.getMotorCoeff(StringUtilities::makeFullyQualifiedRoboDynElement( mechanism,  ApiMap::getXmlElementValue(parametersElement, "SliderMaxVelocityCommand" )));
        double vKp =            io.getMotorCoeff(StringUtilities::makeFullyQualifiedRoboDynElement( mechanism,  ApiMap::getXmlElementValue(parametersElement, "SliderVelocityKpGainCommand" )));
        double vKi =            io.getMotorCoeff(StringUtilities::makeFullyQualifiedRoboDynElement( mechanism,  ApiMap::getXmlElementValue(parametersElement, "SliderVelocityKiGainCommand" )));
        double windup =         io.getMotorCoeff(StringUtilities::makeFullyQualifiedRoboDynElement( mechanism,  ApiMap::getXmlElementValue(parametersElement, "SliderIntegratorWindupLimit" )));

        tempController.setVelocityLoopParameters(minVelocity, maxVelocity, vKp, vKi, windup, bypass <= 0 ? false : true);

        bypass =                io.getMotorCoeff(StringUtilities::makeFullyQualifiedRoboDynElement( mechanism,  ApiMap::getXmlElementValue(parametersElement, "CurrentLoopBypass")));
        double minCurrent =     io.getMotorCoeff(StringUtilities::makeFullyQualifiedRoboDynElement( mechanism,  ApiMap::getXmlElementValue(parametersElement, "SliderMinCurrentCommand" )));
        double maxCurrent =     io.getMotorCoeff(StringUtilities::makeFullyQualifiedRoboDynElement( mechanism,  ApiMap::getXmlElementValue(parametersElement, "SliderMaxCurrentCommand" )));
        double maxDutyCycle =   io.getMotorCoeff(StringUtilities::makeFullyQualifiedRoboDynElement( mechanism,  ApiMap::getXmlElementValue(parametersElement, "SliderMaxDutyCycleCommand" )));

        tempController.setCurrentLoopParameters(minCurrent, maxCurrent, maxDutyCycle, bypass <= 0 ? false : true);

        double motorViscousDampingComp =    io.getMotorCoeff(StringUtilities::makeFullyQualifiedRoboDynElement( mechanism,  ApiMap::getXmlElementValue(parametersElement, "MotorViscousDampingCompensation" )));
        double motorInertia =               io.getMotorCoeff(StringUtilities::makeFullyQualifiedRoboDynElement( mechanism,  ApiMap::getXmlElementValue(parametersElement, "MotorInertia" )));
        double motorTorqueConst =           io.getMotorCoeff(StringUtilities::makeFullyQualifiedRoboDynElement( mechanism,  ApiMap::getXmlElementValue(parametersElement, "MotorTorqueConstant" )));
        double motorPhaseRes =              io.getMotorCoeff(StringUtilities::makeFullyQualifiedRoboDynElement( mechanism,  ApiMap::getXmlElementValue(parametersElement, "MotorPhaseResistance" )));
        double motorBackEMF =               io.getMotorCoeff(StringUtilities::makeFullyQualifiedRoboDynElement( mechanism,  ApiMap::getXmlElementValue(parametersElement, "MotorBackEMFConstant" )));
        double motorImductance =            io.getMotorCoeff(StringUtilities::makeFullyQualifiedRoboDynElement( mechanism,  ApiMap::getXmlElementValue(parametersElement, "MotorInductance" )));
        double motionRatio =                io.getMotorCoeff(StringUtilities::makeFullyQualifiedRoboDynElement( mechanism,  ApiMap::getXmlElementValue(parametersElement, "MotionRatio" )));
        double PWMFreq =                    io.getMotorCoeff(StringUtilities::makeFullyQualifiedRoboDynElement( mechanism,  ApiMap::getXmlElementValue(parametersElement, "PWMFrequency" )));
        double bridgeDeadTime =             io.getMotorCoeff(StringUtilities::makeFullyQualifiedRoboDynElement( mechanism,  ApiMap::getXmlElementValue(parametersElement, "BridgeDeadTime" )));
        double bridgeSwitchTime =           io.getMotorCoeff(StringUtilities::makeFullyQualifiedRoboDynElement( mechanism,  ApiMap::getXmlElementValue(parametersElement, "BridgeSwitchTime" )));

        tempController.setHardwareParameters(motorViscousDampingComp, motorInertia, motorTorqueConst, motorPhaseRes, motorBackEMF, motorImductance, motionRatio, PWMFreq, bridgeDeadTime, bridgeSwitchTime);

        for (unsigned int i = 0; i < N+1; ++i)
        {
            controller.setMultiLoopController(tempController, i);
        }

        for (unsigned int i = 0; i < N; ++i)
        {
            str.str("");
            str << "Joint" << i << "PositionMin";
            jointCapabilityMsg.positionLimitMin[i] = io.getMotorCoeff(StringUtilities::makeFullyQualifiedRoboDynElement( mechanism,  ApiMap::getXmlElementValue(parametersElement, str.str())));
            str.str("");
            str << "Joint" << i << "PositionMax";
            jointCapabilityMsg.positionLimitMax[i] = io.getMotorCoeff(StringUtilities::makeFullyQualifiedRoboDynElement( mechanism,  ApiMap::getXmlElementValue(parametersElement, str.str())));
        }

        /// Reference Matrix
        typename FingerController<N>::ReferenceMatrixType refMatrix;
        for (unsigned int i = 0; i < refMatrix.rows(); ++i)
        {
            for (unsigned int j = 0; j < refMatrix.cols(); ++j)
            {
                str.str("");
                str << "ReferenceMatrix" << i << j;
                refMatrix(i, j) = io.getMotorCoeff(StringUtilities::makeFullyQualifiedRoboDynElement( mechanism,  ApiMap::getXmlElementValue(parametersElement, str.str())));
            }
        }
        controller.setReferenceMatrix(refMatrix);

        //! live coeff
        nameIsCalibrated          = StringUtilities::makeFullyQualifiedRoboDynElement(mechanism, "IsCalibrated");
        kTendonName               = StringUtilities::makeFullyQualifiedRoboDynElement(mechanism, "KTendon");
        io.setLiveCoeff(kTendonName, 0.);
        for (unsigned int i = 0; i< N+1; ++i)
        {
            encoderOffsetElements[i] = StringUtilities::makeFullyQualifiedRoboDynElement(roboDynActuators[i], "EncoderOffset");
            sliderOffsetElements[i] = StringUtilities::makeFullyQualifiedRoboDynElement(roboDynActuators[i], "SliderOffset");
            sliderTarePosElements[i] = StringUtilities::makeFullyQualifiedRoboDynElement(roboDynActuators[i], "SliderTarePosition");
            tensionOffsetElements[i] = StringUtilities::makeFullyQualifiedRoboDynElement(roboDynActuators[i], "TensionOffset");
        }
        io.setLiveCoeff(nameIsCalibrated,          0.);
    }
    else
    {
        std::stringstream err;
        err << "The file " << parameterFile << " has no element named [ApiMap]";
        RCS::Logger::log("gov.nasa.robonet.JointCommandFinger", log4cpp::Priority::ERROR, err.str());
        throw std::runtime_error(err.str());
    }
}

template <unsigned int N>
void JointCommandFinger<N>::updateMeasuredState(nasa_r2_common_msgs::JointControlData msg)
{
    unsigned int i;

    // If coeffs are not loaded, override with zeros RDEV-1075
    if (msg.coeffState.state != nasa_r2_common_msgs::JointControlCoeffState::LOADED)
    {
        unsigned int numStates = 2 * N + 1; // n joints, n+1 sliders (with tension/effort)
        positionVals.assign(numStates, 0.0);
        velocityVals.assign(numStates, 0.0);
        effortVals.assign(numStates, 0.0);
        return;
    }

    for (i = 0; i < encoderStateElements.size(); ++i)
    {
        encoderVec[i] = io.getInt16(encoderStateElements[i]);
    }

    for (i = 0; i < hallStateElements.size(); ++i)
    {
        hallVec[i] = io.getUInt16(hallStateElements[i]);
    }

    for (i = 0; i < tensionAStateElements.size(); ++i)
    {
        tensionAVec[i] = io.getUInt16(tensionAStateElements[i]);
    }

    for (i = 0; i < tensionBStateElements.size(); ++i)
    {
        tensionBVec[i] = io.getUInt16(tensionBStateElements[i]);
    }

    // get hall positions
    controller.getHallAngles(hallVec, hallVec);

    if (io.getLiveCoeff(nameIsCalibrated))
    {
        // calibrated
        if (!tubeTared)
        {
            typename FingerController<N>::SliderVectorType sliderOffset, sliderTarePos, tensionOffset;
            for (i = 0; i < encoderOffset.rows(); ++i)
            {
                encoderOffset[i] = io.getLiveCoeff(encoderOffsetElements[i]);
                sliderOffset[i]  = io.getLiveCoeff(sliderOffsetElements[i]);
                sliderTarePos[i] = io.getLiveCoeff(sliderTarePosElements[i]);
                tensionOffset[i] = io.getLiveCoeff(tensionOffsetElements[i]);
            }
            sliderOffset -= encoderOffset;
            controller.setTubeTareParameters(sliderOffset, sliderTarePos, tensionOffset);
            controller.reset();
            filterInitialized = false;
            tubeTared = true;
            
            //check that tensions are above the minimum tension, or adjust tensionOffset
            controller.getCalibratedTensions(tensionAVec, tensionBVec, 0.0, tensionVec);
            for (i = 0; i < encoderOffset.rows(); ++i)
            {
                if (tensionVec[i] < tensionMin)
                    tensionOffset[i] += tensionVec[i] - tensionMin; //tensionVec.sum()/tensionVec.rows();
            }
            controller.setTubeTareParameters(sliderOffset, sliderTarePos, tensionOffset);
        }

        // get positions
        encoderVec -= encoderOffset;
        controller.getSlidersFromEncoders(encoderVec, sliderVec);

        // filter sliders and halls
        if (!filterInitialized)
        {
            filteredSliderVec = sliderVec;
            filteredHallVec = hallVec;
        }
        filteredSliderVec = filteredSliderVec * positionAlpha + sliderVec * (1 - positionAlpha);
        filteredHallVec = filteredHallVec * positionAlpha + hallVec * (1 - positionAlpha);

        controller.getJointsFromSliders(filteredSliderVec, jointVec);
        
        //use hall sensor data
        if(activeStiffnessOn)
        {
            if(jointVec.rows() == 4)
                jointVec.tail(3) = hallVec.tail(3); //don't use roll joint hall
            else if(jointVec.rows() == 3)
                jointVec = hallVec;
        }
        //don't use secondary finger halls (doesn't use combined proximal/medial joint angles)
        
        // get tensions
        controller.getCalibratedTensions(tensionAVec, tensionBVec, timestep, tensionVec);
        typename FingerController<N>::JointVectorType xyPos, desTorques;
        
        if(useCartesian)
            controller.getXYPos(jointVec, isLeft, xyPos, desTorques);
        else
        {
            xyPos = jointVec;
            desTorques = FingerController<N>::JointVectorType::Zero();
        }
        
        for (i = 0; i < N; ++i)
        {
            positionVals[i] = xyPos[i];
            effortVals[i] = desTorques[i];
        }
        for (i = 0; i < N+1; ++i)
        {
            positionVals[N+i] = filteredSliderVec[i];
            effortVals[N+i]   = tensionVec[i];
        }
    }
    else
    {
        // not calibrated
        tubeTared = false;

        for (i = 0; i < N; ++i)
        {
            positionVals[i] = 0.;
        }
        for (i = 0; i < N+1; ++i)
        {
            positionVals[N+i] = 0.;
            effortVals[N+i]   = 0.;
        }
    }

    // get velocities
    if (!filterInitialized)
    {
        velocityVals.assign(velocityVals.size(), 0.);
        filterInitialized = true;
    }
    else
    {
        for (unsigned int i = 0; i < velocityVals.size(); ++i)
        {
            velocityVals[i] = (positionVals[i] - prevPositionVals[i]) / timestep;
        }
    }
    prevPositionVals = positionVals;
}

template <unsigned int N>
sensor_msgs::JointState JointCommandFinger<N>::getSimpleMeasuredState()
{
    simpleMeasuredStateMsg.position = positionVals;
    simpleMeasuredStateMsg.velocity = velocityVals;
    simpleMeasuredStateMsg.effort   = effortVals;

    simpleMeasuredStateMsg.header.stamp = ros::Time::now();
    return simpleMeasuredStateMsg;
}

template <unsigned int N>
sensor_msgs::JointState JointCommandFinger<N>::getCompleteMeasuredState()
{
    prevSmoothedPos = (smoothedPos - prevSmoothedPos) / timestep;
    unsigned int index = 0;
    for (unsigned int i = 0; i < N; ++i)
    {
        // joint vals
        completeMeasuredStateMsg.position[i] = positionVals[i];
        completeMeasuredStateMsg.velocity[i] = velocityVals[i];
        completeMeasuredStateMsg.effort[i] = effortVals[i];

        // joint embeddedCommands
        index = 2*N+1+i;
        completeMeasuredStateMsg.position[index] = smoothedPos[i];
        completeMeasuredStateMsg.velocity[index] = prevSmoothedPos[i];
    }
    for (unsigned int i = 0; i < N+1; ++i)
    {
        // slider vals
        index = N+i;
        completeMeasuredStateMsg.position[index] = positionVals[index];
        completeMeasuredStateMsg.velocity[index] = velocityVals[index];
        completeMeasuredStateMsg.effort[index] = effortVals[index];

        // slider embeddedCommands
        index = 3*N+1+i;
        completeMeasuredStateMsg.position[index] = desiredSliders[i];
        completeMeasuredStateMsg.velocity[index] = desiredSliderVels[i];
    }

    completeMeasuredStateMsg.header.stamp = ros::Time::now();
    prevSmoothedPos = smoothedPos;
    return completeMeasuredStateMsg;
}

template <unsigned int N>
nasa_r2_common_msgs::JointCommand JointCommandFinger<N>::getCommandedState()
{
    commandedStateMsg.header.stamp = ros::Time::now();
    return commandedStateMsg;
}

template <unsigned int N>
void JointCommandFinger<N>::setFaultState()
{
}

template <unsigned int N>
void JointCommandFinger<N>::setCommand(nasa_r2_common_msgs::JointCommand msg, nasa_r2_common_msgs::JointControlData control)
{

    if (!msg.name.empty() && !msg.desiredPosition.empty())
    {
        if (msg.desiredPositionVelocityLimit.empty())
            msg.desiredPositionVelocityLimit.resize(msg.name.size(), 0.);

        for (unsigned int i = 0; i < msg.name.size(); ++i)
        {
            strVecIt = std::find(commandedStateMsg.name.begin(), commandedStateMsg.name.end(), msg.name[i]);
            if (strVecIt != commandedStateMsg.name.end())
            {
                unsigned int index = strVecIt - commandedStateMsg.name.begin();

                // smooth if in drive and not in actuator
                if (control.controlMode.state == nasa_r2_common_msgs::JointControlMode::DRIVE
                        && control.commandMode.state != nasa_r2_common_msgs::JointControlCommandMode::ACTUATOR)
                {
                    smoothedPos[index] = smoother[index].update(msg.desiredPosition[i], msg.desiredPositionVelocityLimit[i]);
                }

                commandedStateMsg.desiredPosition[index] = msg.desiredPosition[i];
                commandedStateMsg.desiredPositionVelocityLimit[index] = msg.desiredPositionVelocityLimit[i];
            }
        }
    }

    if(control.calibrationMode.state == nasa_r2_common_msgs::JointControlCalibrationMode::ENABLE)
    {
        // do nothing if calibrating (tubetare controls when in this mode)
        return;
    }

    if (control.commandMode.state == nasa_r2_common_msgs::JointControlCommandMode::ACTUATOR)
    {
        for (unsigned int i = 0; i < msg.name.size(); ++i)
        {
            strVecIt = std::find(roboDynActuators.begin(), roboDynActuators.end(), msg.name[i]);
            if (strVecIt != roboDynActuators.end())
            {
                desiredSliders[strVecIt-roboDynActuators.begin()] = msg.desiredPosition[i];
            }
        }
        
        // get equivalent joint command
        controller.getJointsFromSliders(desiredSliders, smoothedPos);
        for (unsigned int i = 0; i < smoothedPos.size(); ++i)
        {
            commandedStateMsg.desiredPosition[i] = smoothedPos[i];
            smoother[i].reset(smoothedPos[i], 0.);
        }
    }
    else
    {
        if(activeStiffnessOn)
        {
            typename FingerController<N>::SliderVectorType sliderVel1;
            //! @todo this doesn't seem right: velocity[N]???
            sliderVel1 = FingerController<N>::SliderVectorType::Map(&(velocityVals[N]));
            controller.getDesiredSlidersStiffness(smoothedPos, jointVec, filteredSliderVec, tensionVec, sliderVel1, useCartesian, isLeft, desiredSliders);
        }
        else
        {
            controller.getDesiredSliders(smoothedPos, filteredSliderVec, desiredSliders);
        }

		//for (unsigned int i = 0; i < msg.name.size(); ++i)
        //{
            //strVecIt = std::find(roboDynJoints.begin(), roboDynJoints.end(), msg.name[i]);
            //if (strVecIt != roboDynJoints.end())
            //{
                //desiredSliders[strVecIt-roboDynJoints.begin()] = msg.desiredPosition[i];
            //}
        //}
		
	}


    if(control.controlMode.state == nasa_r2_common_msgs::JointControlMode::DRIVE)
    {
//        getSimpleMeasuredState(); should be called before setCommand...don't call again to avoid messing up velocities

        // get ktendon
        if(io.hasLiveCoeff(kTendonName))
        {
            ktendon = io.getLiveCoeff(kTendonName);
        }

		//// get desiredSliders
		//if (!control.commandMode.state == nasa_r2_common_msgs::JointControlCommandMode::ACTUATOR)
		//{
			//typename FingerController<N>::JointVectorType joints;
			//joints = FingerController<N>::JointVectorType::Map(commandedStateMsg.desiredPosition.data());
            //controller.getDesiredSliders(joints, filteredSliderVec, desiredSliders);
		//}
        
//        std::cout << desiredSliders[0] << "\t" << desiredSliders[1] << "\t" << desiredSliders[2] << "\t" << desiredSliders[3] << std::endl;

        // get motcoms
        typename FingerController<N>::SliderVectorType sliderMotComs, sliderVel;
        sliderVel = FingerController<N>::SliderVectorType::Map(&(velocityVals[N]));
        controller.getMotComs(desiredSliders,ktendon, filteredSliderVec, tensionVec, sliderVel, sliderMotComs, desiredSliderVels);

        for (unsigned int i = 0; i < sliderMotComs.rows(); ++i)
        {
            io.setInt16(motComCommandElements[i], static_cast<int16_t>(sliderMotComs[i]));
        }
    }
    else
    {
        desiredSliderVels.setZero();
        //! we are not in drive, so set motcoms to 0
        for (unsigned int i = 0; i < motComCommandElements.size(); ++i)
        {
            io.setInt16(motComCommandElements[i], 0);
        }
    }
}

template <unsigned int N>
nasa_r2_common_msgs::JointCapability JointCommandFinger<N>::getCapability()
{
    return jointCapabilityMsg;
}


#endif
