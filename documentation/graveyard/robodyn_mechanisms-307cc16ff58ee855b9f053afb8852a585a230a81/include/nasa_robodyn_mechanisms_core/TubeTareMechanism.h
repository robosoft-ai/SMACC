#ifndef TUBE_TARE_UTILITIES_H
#define TUBE_TARE_UTILITIES_H

#include <boost/array.hpp>
#include "nasa_robodyn_mechanisms_core/TubeTareInterface.h"
#include "nasa_r2_config_core/StringUtilities.h"
#include "nasa_robodyn_mechanisms_core/FingerController.h"

template<unsigned int N>
class TubeTareMechanism : public TubeTareInterface
{
public:
    TubeTareMechanism(const std::string& mechanism, TubeTareInterface::IoFunctions ioFunctions, float freq);
    virtual ~TubeTareMechanism();

    virtual void setStop(const std::vector<std::string>& actuators = std::vector<std::string>()); //the motcom command for stop (0)
    virtual bool isMoving(); //checks if the velocity is 0
    virtual void setRelease(const std::vector<std::string>& actuators = std::vector<std::string>()); //the motcom command for release (limited)
    virtual void getEncoderTarePos(); //perform the tare (TareEncoderAbsolute)
    virtual void setTighten(const std::vector<std::string>& actuators = std::vector<std::string>()); //set motcom to tighten position (limited)
    virtual int  goodPosition(); //checks if near end of travel
    virtual void getSliderTarePos();//SliderVectorType &encoderOffsets, SliderVectorType &sliderOffsets);
    virtual void getTensionTareValue();
    virtual void resetTensionCounter();

    FingerController<N> finger;
    typedef typename FingerController<N>::SliderVectorType      SliderVectorType;
    typedef typename FingerController<N>::ReferenceMatrixType   ReferenceMatrixType;
    typedef typename FingerController<N>::JointVectorType       hallVec;

private:
//    FingerController<N> finger;
//    typedef typename FingerController<N>::SliderVectorType      SliderVectorType;
//    typedef typename FingerController<N>::ReferenceMatrixType   ReferenceMatrixType;
//    typedef typename FingerController<N>::JointVectorType       hallVec;


    boost::array<std::string, N>            hallStateElements;
    boost::array<std::string, N+1>          encoderStateElements;
    boost::array<std::string, N+1>          tensionAStateElements;
    boost::array<std::string, N+1>          tensionBStateElements;
    boost::array<std::string, N+1>          motComCommandElements;

    float encoderVelZeroThreshold, encoderMinTravel, encoderMaxTravel;

    SliderVectorType motComReleaseVal, motComTightenVal;
    SliderVectorType tensionGainA, tensionGainB, tensionSensorCalOffset, tension_out;

    boost::array<std::string, N+1>          encoderOffsetElements;
    boost::array<std::string, N+1>          sliderOffsetElements;
    boost::array<std::string, N+1>          sliderTarePosElements;
    boost::array<std::string, N+1>          tensionOffsetElements;

    // filter values
    typename FingerController<N>::SliderVectorType encoderVec;
    typename FingerController<N>::SliderVectorType filteredEncoderVec;
    typename FingerController<N>::SliderVectorType encoderVels;
    typename FingerController<N>::SliderVectorType prevEncoderVec;
    double timestep;
    double positionAlpha;
    bool filterInitialized;

    unsigned int tensionCounter;

    void setParameters();

};

template <unsigned int N>
TubeTareMechanism<N>::TubeTareMechanism(const std::string& mechanism, TubeTareInterface::IoFunctions ioFunctions, float freq)
    : TubeTareInterface(mechanism, ioFunctions), filterInitialized(false)
{
    if (mechanism == "")
    {
        std::stringstream err;
        err << "Constructor requires mechanism be non-empty.";
        RCS::Logger::log("gov.nasa.robonet.TubeTareMechanism", log4cpp::Priority::FATAL, err.str());
        throw std::invalid_argument(err.str());
    }

    if (freq <= 0.)
    {
        std::stringstream err;
        err << "frequency must be positive";
        RCS::Logger::log("gov.nasa.robonet.TubeTareMechanism", log4cpp::Priority::FATAL, err.str());
        throw std::invalid_argument(err.str());
    }

    if (io.getUInt16.empty() or
        io.getInt16.empty() or
        io.setInt16.empty() or
        io.getMotorCoeff.empty() or
        io.hasLiveCoeff.empty() or
        io.getLiveCoeff.empty() or
        io.setLiveCoeff.empty() or
        io.getJointNames.empty() or
        io.getActuatorNames.empty() or
        io.getCommandFile.empty())
    {
        std::stringstream err;
        err << "Constructor requires 'io.getUInt16', 'io.setUInt16', 'io.getMotorCoeff', 'io.hasLiveCoeff', 'io.getLiveCoeff', 'io.setLiveCoeff', io.getJointNames', 'io.getActuatorNames', and 'io.getCommandFile' be non-empty.";
        RCS::Logger::log("gov.nasa.robonet.TubeTareMechanism", log4cpp::Priority::FATAL, err.str());
        throw std::invalid_argument(err.str());
    }

    roboDynJoints    = io.getJointNames(mechanism);
    roboDynActuators = io.getActuatorNames(mechanism);

    if(roboDynJoints.size() != N or
       roboDynActuators.size() != N+1)
    {
        std::stringstream err;
        err << "Constructor requires roboDynJoints have N and roboDynActuators have N+1 values, with N for this instance being " << N << std::endl;
        err << "Number of roboDynJoints: " << roboDynJoints.size() << std::endl;
        err << "Number of roboDynActuators: " << roboDynActuators.size() << std::endl;
        RCS::Logger::log("gov.nasa.robonet.TubeTareMechanism", log4cpp::Priority::FATAL, err.str());
        throw std::invalid_argument(err.str());

    }

    timestep = 1/freq;
    this->setParameters();
}

template <unsigned int N>
TubeTareMechanism<N>::~TubeTareMechanism()
{}

template <unsigned int N>
void TubeTareMechanism<N>::setParameters()
{
    std::string parameterFile = io.getCommandFile(mechanism);

    //! Parse parameter file
    TiXmlDocument file(parameterFile.c_str());
    bool loadOkay = file.LoadFile();
    if (!loadOkay)
    {
        std::stringstream err;
        err << "Failed to load file [" << parameterFile << "]";
        RCS::Logger::log("gov.nasa.robonet.TubeTareMechanism", log4cpp::Priority::FATAL, err.str());
        throw std::runtime_error(err.str());
    }
    TiXmlHandle doc(&file);
    RCS::Logger::log("gov.nasa.robonet.TubeTareMechanism", log4cpp::Priority::INFO, "CommandFile [" + parameterFile + "] successfully loaded.");

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

        encoderVelZeroThreshold =   io.getMotorCoeff(StringUtilities::makeFullyQualifiedRoboDynElement( mechanism, ApiMap::getXmlElementValue(parametersElement, "EncoderVelZeroThreshold")));
        encoderMinTravel =          io.getMotorCoeff(StringUtilities::makeFullyQualifiedRoboDynElement( mechanism, ApiMap::getXmlElementValue(parametersElement, "EncoderMinTravel")));
        encoderMaxTravel =          io.getMotorCoeff(StringUtilities::makeFullyQualifiedRoboDynElement( mechanism, ApiMap::getXmlElementValue(parametersElement, "EncoderMaxTravel")));

        //motcom coeffs
        for (unsigned int i=0; i< motComReleaseVal.rows(); ++i)
        {
            str.str("");
            str << "Actuator" << i << "MotComReleaseVal";
            motComReleaseVal[i] = io.getMotorCoeff(StringUtilities::makeFullyQualifiedRoboDynElement( mechanism,  ApiMap::getXmlElementValue(parametersElement, str.str())));
            str.str("");
            str << "Actuator" << i << "MotComTightenVal";
            motComTightenVal[i] = io.getMotorCoeff(StringUtilities::makeFullyQualifiedRoboDynElement( mechanism,  ApiMap::getXmlElementValue(parametersElement, str.str())));
            str.str("");
        }
        
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
        finger.setHallAngleParameters(hallScale, hallCoeffs0, hallCoeffs1, hallCoeffs2, hallCoeffs3);

        // tension sensor coeffs
        for( unsigned int i=0; i<N+1; i++)
        {
			str.str("");
            str << "Actuator" << i << "TensionAGain";
            tensionGainA[i] = io.getMotorCoeff(StringUtilities::makeFullyQualifiedRoboDynElement( mechanism,  ApiMap::getXmlElementValue(parametersElement, str.str())));
            str.str("");
            str << "Actuator" << i << "TensionBGain";
            tensionGainB[i] = io.getMotorCoeff(StringUtilities::makeFullyQualifiedRoboDynElement( mechanism,  ApiMap::getXmlElementValue(parametersElement, str.str())));
            str.str("");
            str << "Actuator" << i << "TensionSensorCalOffset";
            tensionSensorCalOffset[i] = io.getMotorCoeff(StringUtilities::makeFullyQualifiedRoboDynElement( mechanism,  ApiMap::getXmlElementValue(parametersElement, str.str())));
            tension_out[i] = 0;
		}


        //Output elements
        for (unsigned int i = 0; i< N+1; ++i)
        {
            encoderOffsetElements[i] = StringUtilities::makeFullyQualifiedRoboDynElement(roboDynActuators[i], "EncoderOffset");
            sliderOffsetElements[i] = StringUtilities::makeFullyQualifiedRoboDynElement(roboDynActuators[i], "SliderOffset");
            sliderTarePosElements[i] = StringUtilities::makeFullyQualifiedRoboDynElement(roboDynActuators[i], "SliderTarePosition");
            tensionOffsetElements[i] = StringUtilities::makeFullyQualifiedRoboDynElement(roboDynActuators[i], "TensionOffset");
        }

        // filter
        positionAlpha = io.getMotorCoeff(StringUtilities::makeFullyQualifiedRoboDynElement( mechanism,  ApiMap::getXmlElementValue(parametersElement, "PositionAlpha")));

        // tension counter
        tensionCounter = 0;

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
        finger.setReferenceMatrix(refMatrix);
    }
    else
    {
        std::stringstream err;
        err << "The file " << parameterFile << " has no element named [ApiMap]";
        RCS::Logger::log("gov.nasa.robonet.TubeTareMechanism", log4cpp::Priority::ERROR, err.str());
        throw std::runtime_error(err.str());
    }
}


template <unsigned int N>
void TubeTareMechanism<N>::setStop(const std::vector<std::string>& actuators)
{
    //if no actuators sent in, do them all
    if(actuators.empty())
    {
        std::stringstream err;
        err << "Stopping all actuators for " << mechanism;
        RCS::Logger::log("gov.nasa.robonet.TubeTareMechanism", log4cpp::Priority::DEBUG, err.str());
        for (unsigned int i = 0; i < motComCommandElements.size(); ++i)
        {
            io.setInt16(motComCommandElements[i], static_cast<int16_t>(0));
        }
    }
    else
    {
        for(unsigned int i =0; i< roboDynActuators.size(); ++i)
        {
            if(std::find(actuators.begin(), actuators.end(), roboDynActuators[i]) != actuators.end() )
            {
				std::stringstream err;
                err << "Stopping " << mechanism << " actuator: " << roboDynActuators[i] <<" motcomCommandElement: "<< motComCommandElements[i];
				RCS::Logger::log("gov.nasa.robonet.TubeTareMechanism", log4cpp::Priority::DEBUG, err.str());
                io.setInt16(motComCommandElements[i], static_cast<int16_t>(0));
            }
        }
    }
}

template <unsigned int N>
bool TubeTareMechanism<N>::isMoving()
{
    for(unsigned int i =0; i<encoderStateElements.size(); ++i)
    {
        encoderVec[i] = io.getInt16(encoderStateElements[i]);
    }

    // filter
    if (!filterInitialized)
    {
        filteredEncoderVec = encoderVec;
        encoderVels.setZero();
        filterInitialized = true;
    }
    else
    {
        filteredEncoderVec = filteredEncoderVec * positionAlpha + encoderVec * (1 - positionAlpha);
        encoderVels = (filteredEncoderVec - prevEncoderVec) / timestep;
    }
    prevEncoderVec = filteredEncoderVec;

    for(unsigned int i = 0; i< encoderVels.size(); ++i)
    {
        if(-encoderVelZeroThreshold > encoderVels[i] || encoderVels[i] > encoderVelZeroThreshold)
        {
            return true;
        }
    }

    return false;
}

template <unsigned int N>
void TubeTareMechanism<N>::setRelease(const std::vector<std::string>& actuators)
{
    if(actuators.empty())
    {
		std::stringstream err;
		err << "Set release for all actuators in " << mechanism;
        for (unsigned int i = 0; i < motComCommandElements.size(); ++i )
        {
			err << "\n\t" << motComCommandElements[i] << ": " << static_cast<int16_t>(motComReleaseVal[i]);
            io.setInt16(motComCommandElements[i], static_cast<int16_t>(motComReleaseVal[i]) );
        }
		RCS::Logger::log("gov.nasa.robonet.TubeTareMechanism", log4cpp::Priority::DEBUG, err.str());
    }
    else
    {
        for(unsigned int i =0; i< roboDynActuators.size(); ++i)
        {
            if(std::find(actuators.begin(), actuators.end(), roboDynActuators[i]) != actuators.end() )
            {
                std::stringstream err;
                err << "Set release for " << mechanism << " actuator: " << roboDynActuators[i] <<" motcomCommandElement: "<< motComCommandElements[i];
                RCS::Logger::log("gov.nasa.robonet.TubeTareMechanism", log4cpp::Priority::DEBUG, err.str());
                io.setInt16(motComCommandElements[i], static_cast<int16_t>(motComReleaseVal[i]));
            }
        }
    }
    filterInitialized = false;
}


template <unsigned int N>
void TubeTareMechanism<N>::getEncoderTarePos()
{
    for(unsigned int i = 0; i<encoderOffsetElements.size(); ++i)
    {
        io.setLiveCoeff(encoderOffsetElements[i], io.getInt16(encoderStateElements[i]));
    }
}

template <unsigned int N>
void TubeTareMechanism<N>::setTighten(const std::vector<std::string>& actuators)
{
    if(actuators.empty())
    {
		std::stringstream err;
		err << "Set tighten for all actuators in " << mechanism;
        for (unsigned int i = 0; i < motComCommandElements.size(); ++i )
        {
			err << "\n\t" << motComCommandElements[i] << ": " << static_cast<int16_t>(motComTightenVal[i]);
            io.setInt16(motComCommandElements[i], static_cast<int16_t>(motComTightenVal[i]) );
        }
		RCS::Logger::log("gov.nasa.robonet.TubeTareMechanism", log4cpp::Priority::DEBUG, err.str());
    }
    else
    {
        //std::cout<<"Set tighten on list"<<std::endl;
        for(unsigned int i =0; i< roboDynActuators.size(); ++i)
        {
            //std::cout<<"looking for actuator: "<< roboDynActuators[i]<<std::endl;
            if(std::find(actuators.begin(), actuators.end(), roboDynActuators[i]) != actuators.end() )
            {
                std::stringstream err;
                err << "Set tighten for " << mechanism << " actuator: " << roboDynActuators[i] <<" motcomCommandElement: "<< motComCommandElements[i];
                RCS::Logger::log("gov.nasa.robonet.TubeTareMechanism", log4cpp::Priority::DEBUG, err.str());
                io.setInt16(motComCommandElements[i], static_cast<int16_t>(motComTightenVal[i]));
            }
//            else
//            {
//				std::cout<<"actuator not found"<<std::endl;
//			}
        }
    }
    filterInitialized = false;
}

template <unsigned int N>
int TubeTareMechanism<N>::goodPosition()
{

    for(unsigned int i = 0; i < encoderStateElements.size(); ++i)
    {
        int encoderAbsolute = io.getInt16(encoderStateElements[i]) - io.getLiveCoeff(encoderOffsetElements[i]);

        if(-encoderMinTravel < encoderAbsolute && encoderAbsolute < encoderMinTravel)
        {
            return -1;
        }
        if(encoderAbsolute < -encoderMaxTravel || encoderAbsolute > encoderMaxTravel)
        {
            return 1;
        }
    }

    return 0;

}

template<unsigned int N>
void TubeTareMechanism<N>::getSliderTarePos()
{
    hallVec currentHalls;
    SliderVectorType sliderPos;
    for(unsigned int i = 0; i< hallStateElements.size(); ++i)
    {
        currentHalls[i] = io.getUInt16(hallStateElements[i]);
    }

    finger.getHallAngles(currentHalls, currentHalls);
    finger.getSlidersFromJoints(currentHalls, sliderPos);

    for(unsigned int i = 0; i<sliderPos.size(); ++i)
    {
        io.setLiveCoeff(sliderOffsetElements[i], io.getInt16(encoderStateElements[i]));
        io.setLiveCoeff(sliderTarePosElements[i], sliderPos[i]);
    }

}

template<unsigned int N>
void TubeTareMechanism<N>::getTensionTareValue()
{
	SliderVectorType tensionA, tensionB;
	
	tensionCounter++;
	
	for(unsigned int i=0; i<tensionAStateElements.size(); i++)
	{
		tensionA[i] = io.getUInt16(tensionAStateElements[i]);
		tensionB[i] = io.getUInt16(tensionBStateElements[i]);
	}
	
	for(unsigned int i=0; i<tension_out.size(); i++)
	{
		// find "calibrated" value
		double temp = tensionA[i] * tensionGainA[i] + tensionB[i] * tensionGainB[i] + tensionSensorCalOffset[i];
		
		// take running average
		tension_out[i] = ((tensionCounter-1)*tension_out[i] + temp)/tensionCounter;
		
		// send it out into the world
		io.setLiveCoeff(tensionOffsetElements[i], tension_out[i]);
	}
	
}

template<unsigned int N>
void TubeTareMechanism<N>::resetTensionCounter()
{
	tensionCounter = 0;
	for(unsigned int i=0; i<tension_out.size(); i++)
	{
		tension_out[i] = 0;
	}
}

#endif // TUBE_TARE_UTILITIES_H
