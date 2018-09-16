/**
 * @file FingerController.h
 * @brief Defines the FingerController class.
 * @author Ross Taylor
 * @date 2013-03-04
 */
#ifndef FINGERCONTROLLER_H
#define FINGERCONTROLLER_H

#include "nasa_common_utilities/Logger.h"
#include <eigen3/Eigen/Dense>
#include "nasa_robodyn_mechanisms_core/FingerKinematics.h"
#include <boost/array.hpp>
#include "nasa_robodyn_utilities/MultiLoopController.h"
#include "nasa_robodyn_mechanisms_core/HallsToAngle.h"

template<unsigned int N>
class FingerController
{
public:
    typedef typename FingerKinematics<N>::ReferenceMatrixType   ReferenceMatrixType;
    typedef typename FingerKinematics<N>::JointVectorType       JointVectorType;
    typedef typename FingerKinematics<N>::SliderVectorType      SliderVectorType;
    typedef typename FingerKinematics<N>::JacobianType      	JacobianType;

    typedef boost::array<MultiLoopController, N+1>              MultiLoopVectorType;

    FingerController();
    virtual ~FingerController() {}

    void setReferenceMatrix(const ReferenceMatrixType& refMatrix_in)
    {
        kinematics.setReferenceMatrix(refMatrix_in);
    }

    /**
     * @brief reset integrators
     */
    void reset();
    /**
     * @brief get limited desiredSliders
     */
    void getDesiredSliders(const JointVectorType& desiredJoints, const SliderVectorType& actualSliders, SliderVectorType& desiredSliders);
    /**
     * @brief get desiredSliders for joint stiffness control
     */
    void getDesiredSlidersStiffness(const JointVectorType& desiredPos, const JointVectorType& actualJoints, const SliderVectorType& actualSliders, const SliderVectorType& actualTensions, const SliderVectorType& sliderVelocities, const bool useCartesian, const bool isLeft, SliderVectorType& desiredSliders);
    /**
     * @brief update calculate motcoms to achieve desiredSliders
     */
    void getMotComs(const SliderVectorType& desiredSliders, const double& ktendon, const SliderVectorType& actualSliders, const SliderVectorType& actualTensions, const SliderVectorType& sliderVelocities, SliderVectorType& sliderMotComs, SliderVectorType& desiredSliderVels);

    // set parameters
    void setMillimetersPerCount(double mmPerCount_in) {mmPerCount = mmPerCount_in;}
    void setHallAngleParameters(double scale, const JointVectorType& coeffs0, const JointVectorType& coeffs1, const JointVectorType& coeffs2, const JointVectorType& coeffs3);
    void setTensionParameters(const SliderVectorType& aGains, const SliderVectorType& bGains, const SliderVectorType& offsets, const SliderVectorType& calibratedStrain_in);
    /**
     * @brief setTubeTareParameters
     * @param encoderOffsets_in encoder value at encoder zero (end of travel)
     * @param sliderOffsets_in encoder value at slider tare position (end of tube tare)
     * @param sliderTarePos_in slider tare position (mm at end of tube tare)
     * @param tensionOffsets_in at tension zero (end of travel)
     */
    void setTubeTareParameters(const SliderVectorType& sliderOffsets_in,
                               const SliderVectorType& sliderTarePos_in, const SliderVectorType& tensionOffsets_in);
    void setStiffnessGains(const JointVectorType& K_in, const double kd_in, const double fmin_in, const double fmax_in, const JointVectorType& jointKp_in, const double tensionKp_in, const double tFreq, const double deadband_in);
    void setMultiLoopController(const MultiLoopController& mlc_in, unsigned int controllerIndex);
    void setBusVoltage(double busVoltage_in) {busVoltage = busVoltage_in;}

    void getHallAngles(const JointVectorType& halls_in, JointVectorType& jointAngles);
    void getCalibratedTensions(const SliderVectorType& tensionA, const SliderVectorType& tensionB, double timestep, SliderVectorType& tensionsCalibrated);
    void getXYPos(const JointVectorType& jointAngles, const bool isLeft, JointVectorType& xy, JointVectorType& desiredTorques);

    inline void getSlidersFromJoints(const JointVectorType& jointAngles, SliderVectorType& sliderPositions)
    {
        kinematics.inverse(jointAngles, sliderPositions);
    }

    inline void getSlidersFromEncoders(const SliderVectorType& encoderPositions, SliderVectorType& sliderPositions)
    {
        sliderPositions = ((encoderPositions - sliderOffsets) * mmPerCount + sliderTarePos) ;
    }

    inline void getJointsFromSliders(const SliderVectorType& sliderPositions, JointVectorType& jointAngles)
    {
        kinematics.forward(sliderPositions, jointAngles);
    }

private:
    FingerKinematics<N> kinematics;
    double              mmPerCount;
    double              hallScale;
    JointVectorType     hallCoeffs0;
    JointVectorType     hallCoeffs1;
    JointVectorType     hallCoeffs2;
    JointVectorType     hallCoeffs3;
    SliderVectorType    tensionAGains;
    SliderVectorType    tensionBGains;
    SliderVectorType    tensionSensorCalOffsets;
    SliderVectorType    calibratedStrain;
    SliderVectorType    sliderOffsets;
    SliderVectorType    sliderTarePos;
    SliderVectorType    tensionOffsets;
    MultiLoopVectorType multiLoopControllers;
    double              smoothingHz;
    SliderVectorType    smoothingFactors;
    double              busVoltage;
    JointVectorType     K;
    double              kd;
    double              fmin;
    double              fmax;
    JointVectorType     jointKp;
    double              tensionKp;
    SliderVectorType    tensionsPrev;
    double              tensionFilterCutoffFreq;
    double              deadbandSize;
    JointVectorType     desTorques;
};

template<unsigned int N>
FingerController<N>::FingerController()
    : mmPerCount(0.0174427)
    , hallScale(0.)
    , hallCoeffs0(JointVectorType::Zero())
    , hallCoeffs1(JointVectorType::Zero())
    , hallCoeffs2(JointVectorType::Zero())
    , hallCoeffs3(JointVectorType::Zero())
    , tensionAGains(SliderVectorType::Zero())
    , tensionBGains(SliderVectorType::Zero())
    , tensionSensorCalOffsets(SliderVectorType::Zero())
    , calibratedStrain(SliderVectorType::Zero())
    , sliderOffsets(SliderVectorType::Zero())
    , sliderTarePos(SliderVectorType::Zero())
    , tensionOffsets(SliderVectorType::Zero())
    , busVoltage(48)
    , K(JointVectorType::Zero())
    , kd(0.01)
    , fmin(1.0)
    , fmax(15.0)
    , jointKp(JointVectorType::Zero())
    , tensionKp(0.003)
    , tensionsPrev(SliderVectorType::Zero())
    , tensionFilterCutoffFreq(20.)
    , deadbandSize(0.5)
    , desTorques(JointVectorType::Zero())
{
    reset();
}

template<unsigned int N>
void FingerController<N>::setHallAngleParameters(double scale, const JointVectorType& coeffs0, const JointVectorType& coeffs1, const JointVectorType& coeffs2, const JointVectorType& coeffs3)
{
    hallScale = scale;
    hallCoeffs0 = coeffs0;
    hallCoeffs1 = coeffs1;
    hallCoeffs2 = coeffs2;
    hallCoeffs3 = coeffs3;
}

template<unsigned int N>
void FingerController<N>::setTensionParameters(const SliderVectorType& aGains, const SliderVectorType& bGains, const SliderVectorType& offsets, const SliderVectorType& calibratedStrain_in)
{
	tensionAGains = aGains;
	tensionBGains = bGains;
	tensionSensorCalOffsets = offsets;
	calibratedStrain = calibratedStrain_in;
}

template<unsigned int N>
void FingerController<N>::setTubeTareParameters(const SliderVectorType& sliderOffsets_in,
                                                const SliderVectorType& sliderTarePos_in, const SliderVectorType& tensionOffsets_in)
{
    sliderOffsets = sliderOffsets_in;
    tensionOffsets = tensionOffsets_in;

    /// round sliderTarePos to nearest mmPerCount
    double roundedSlider;
    for (unsigned int i = 0; i < sliderTarePos_in.size(); ++i)
    {
        roundedSlider = fmod(sliderTarePos_in[i], mmPerCount);
        if (roundedSlider < 0.5)
        {
            roundedSlider = sliderTarePos_in[i] - roundedSlider;
        }
        else
        {
            roundedSlider = sliderTarePos_in[i] + mmPerCount - roundedSlider;
        }

        sliderTarePos[i] = roundedSlider;
    }

}

template<unsigned int N>
void FingerController<N>::setMultiLoopController(const MultiLoopController &mlc_in, unsigned int controllerIndex)
{
    if (controllerIndex > N)
    {
        std::stringstream err;
        err << "setMultiLoopController() controllerIndex out of range" << std::endl;
        RCS::Logger::log("gov.nasa.controllers.FingerController", log4cpp::Priority::ERROR, err.str());
        throw std::runtime_error(err.str());
        return;
    }
    multiLoopControllers[controllerIndex] = mlc_in;
}

template<unsigned int N>
void FingerController<N>::setStiffnessGains(const JointVectorType& K_in, const double kd_in, const double fmin_in, const double fmax_in, const JointVectorType& jointKp_in, const double tensionKp_in, const double tFreq, const double deadband_in)
{
    K = K_in;
    kd = kd_in;
    fmin = fmin_in;
    fmax = fmax_in;
    jointKp = jointKp_in;
    tensionKp = tensionKp_in;
    tensionFilterCutoffFreq = tFreq;
    deadbandSize = deadband_in;
}

template<unsigned int N>
void FingerController<N>::reset()
{
    for (unsigned int i = 0; i < multiLoopControllers.size(); ++i)
    {
        multiLoopControllers[i].reset();
    }
}


//Position control
template<unsigned int N>
void FingerController<N>::getDesiredSliders(const JointVectorType& desiredJoints, const SliderVectorType& actualSliders, SliderVectorType& desiredSliders)
{
    SliderVectorType rsSliders;
    getSlidersFromJoints(desiredJoints, desiredSliders);

    // limit desired
    kinematics.projectToRangeSpace(actualSliders, rsSliders);
    for (int i = 0; i < desiredSliders.rows(); ++i)
    {
        if (desiredSliders[i] > rsSliders[i] + 2.54)
        {
            desiredSliders[i] = rsSliders[i] + 2.54;
        }
        else if (desiredSliders[i] < rsSliders[i] - 2.54)
        {
            desiredSliders[i] = rsSliders[i] - 2.54;
        }
    }
}

//Stiffness Control
template<unsigned int N>
void FingerController<N>::getDesiredSlidersStiffness(const JointVectorType& desiredPos, const JointVectorType& actualJoints, const SliderVectorType& actualSliders, const SliderVectorType& actualTensions, const SliderVectorType& sliderVelocities, const bool useCartesian, const bool isLeft, SliderVectorType& desiredSliders)
{
    //Needed variables -> K, kd, fmin, fmax, jointKp, tensionKp
    
    //find actual torques from tension sensors
    JointVectorType actualTorques;
    kinematics.tensionToJointTorque(actualTensions, actualTorques);

    JointVectorType desiredTorques;

    if(useCartesian && actualJoints.rows() >= 3)
    {
        //find current XY position
        JointVectorType xyActual;
        kinematics.jointToCartesian(actualJoints, isLeft, xyActual);
        
        //calculate Jacobian
        JacobianType jacobian;
        kinematics.calcJacobian(actualJoints, xyActual, isLeft, jacobian);

        //limit desired positions to be within reach
        JointVectorType desiredForces = desiredPos;
        if(desiredForces.rows() == 3) //limit primary fingers
        {
            if(desiredForces[2] < 0)
                desiredForces[2] = 0;
            double dist = sqrt(desiredForces[1]*desiredForces[1]+desiredForces[2]*desiredForces[2]);
            if(dist > 90.0)
                desiredForces.tail(2) *= 90.0/dist;
        }
        
        desiredForces = K.asDiagonal() * (desiredForces - xyActual);

        kinematics.forceToJointTorque(jacobian, desiredForces, desiredTorques);
        //add in repulsive torque to avoid singularity
        if(actualJoints.rows() == 3 && actualJoints[2] < 0.5)
            desiredTorques[2] += 500.0*(0.5-actualJoints[2]);
        //keep thumbs away from joint limits
        if(actualJoints.rows()==4)
        {
            if(actualJoints[0] > -0.3)
                desiredTorques[0] -= 500.0*(actualJoints[0]+0.3);
            else if(actualJoints[0] < -1.4)
                desiredTorques[0] += 500.0*(-1.4-actualJoints[0]);
            if(actualJoints[1] < 0.2)
                desiredTorques[1] += 500.0*(0.2-actualJoints[1]);
            else if(actualJoints[1] > 1.4)
                desiredTorques[1] -= 500.0*(actualJoints[1]-1.4);
            if(actualJoints[2] < 0.5)
                desiredTorques[2] += 500.0*(0.5-actualJoints[2]);
            else if(actualJoints[2] > 1.4)
                desiredTorques[2] -= 500.0*(actualJoints[2]-1.4);
        }
    }
    else
    {
        desiredTorques = K.asDiagonal() * (desiredPos - actualJoints);
    }
    
    desTorques = desiredTorques;
        
    //find internal tension and scale desired torques if necessary
    double tensErr = kinematics.findDesiredInternalTension(fmin, fmax, desiredTorques);
    tensErr -= kinematics.calculateInternalTension(actualTensions);
    
    //subtract to make desiredTorques into torqueError to use in feedback
    desiredTorques -= actualTorques;
    //compute force feedback term -> xd = P^T K_p * torqueError
    kinematics.jointSpaceTorqueFeedback(desiredTorques, jointKp, tensErr, tensionKp, desiredSliders);
    
    //add in slider position and velocity terms
    desiredSliders += actualSliders - kd * sliderVelocities;

    //deadband to reduce vibrations
    for (unsigned int i = 0; i < desiredSliders.rows(); ++i)
        if(desiredSliders[i] < actualSliders[i] + deadbandSize && desiredSliders[i] > actualSliders[i] - deadbandSize)
            desiredSliders[i] = actualSliders[i];
}

template<unsigned int N>
void FingerController<N>::getMotComs(const SliderVectorType& desiredSliders, const double& ktendon, const SliderVectorType& actualSliders, const SliderVectorType& actualTensions, const SliderVectorType& sliderVelocities, SliderVectorType& sliderMotComs, SliderVectorType& desiredSliderVels)
{
    // get motcoms
    double roundedSlider;
    for (unsigned int i = 0; i < multiLoopControllers.size(); ++i)
    {
        roundedSlider = fmod(desiredSliders[i], mmPerCount);
        if (roundedSlider < 0.5)
        {
            roundedSlider = desiredSliders[i] - roundedSlider;
        }
        else
        {
            roundedSlider = desiredSliders[i] + mmPerCount - roundedSlider;
        }

        sliderMotComs[i] = multiLoopControllers[i].update(roundedSlider, ktendon, actualSliders[i], actualTensions[i], sliderVelocities[i], busVoltage, desiredSliderVels[i]);
    }
}

template<unsigned int N>
void FingerController<N>::getHallAngles(const JointVectorType& halls_in, JointVectorType& jointAngles)
{
    for (int i = 0; i < halls_in.rows(); ++i)
    {
        jointAngles[i] = HallsToAngle::getAngleFromHalls(halls_in[i], hallCoeffs0[i], hallCoeffs1[i], hallCoeffs2[i], hallCoeffs3[i], hallScale);
    }
}

template<unsigned int N>
void FingerController<N>::getCalibratedTensions(const SliderVectorType& tensionA, const SliderVectorType& tensionB, double timestep, SliderVectorType& tensionsCalibrated)
{
    for(int i=0; i < tensionA.size(); i++)
    {
        double temp;
        temp = tensionA[i] * tensionAGains[i] + tensionB[i] * tensionBGains[i] + tensionSensorCalOffsets[i];
        tensionsCalibrated[i] = calibratedStrain[i] * (temp - tensionOffsets[i]);
        if(tensionsCalibrated[i] < 0)
            tensionsCalibrated[i] = 0;
        if(tensionFilterCutoffFreq != 0 && timestep != 0.)
        {
            //first-order filter
            temp = tensionFilterCutoffFreq*(tensionsCalibrated[i]-tensionsPrev[i]);
            tensionsCalibrated[i] = tensionsPrev[i] + temp * timestep;
            tensionsPrev[i] = tensionsCalibrated[i];
        }
    }
}

template<unsigned int N>
void FingerController<N>::getXYPos(const JointVectorType& jointAngles, const bool isLeft, JointVectorType& xy, JointVectorType& desiredTorques)
{
	kinematics.jointToCartesian(jointAngles, isLeft, xy);
    desiredTorques = desTorques;
}

#endif
