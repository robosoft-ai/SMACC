/**
 * @file FingerKinematics.h
 * @brief Defines the FingerKinematics class.
 * @author Ross Taylor
 * @date 2013-02-28
 */
#ifndef FINGERKINEMATICS_H
#define FINGERKINEMATICS_H

#include "nasa_common_utilities/Logger.h"
#include <eigen3/Eigen/Dense>
#include <kdl/utilities/svd_eigen_HH.hpp>

template<unsigned int N>
class FingerKinematics
{    
    std::stringstream str;
public:
    typedef Eigen::Matrix<double, N, 1>   JointVectorType;
    typedef Eigen::Matrix<double, N+1, 1> SliderVectorType;
    typedef Eigen::Matrix<double, N, N+1> ReferenceMatrixType;
    typedef Eigen::Matrix<double, N+1, N> ReferenceMatrixTransposeType;
    typedef Eigen::Matrix<double, N, N+1> ReferenceMatrixTransposeInverseType;
    typedef Eigen::Matrix<double, N, N>   JacobianType;

    FingerKinematics();
    virtual ~FingerKinematics() {}

    void setReferenceMatrix(const ReferenceMatrixType& refMatrix_in);

    void forward(const SliderVectorType& sliderPositions, JointVectorType& jointPositions);
    void inverse(const JointVectorType& jointPositions, SliderVectorType& sliderPositions);
    void projectToRangeSpace(const SliderVectorType& sliderPositions_in, SliderVectorType& sliderPositions_out);
    void tensionToJointTorque(const SliderVectorType& actualTensions, JointVectorType& jointTorques);
    double calculateInternalTension(const SliderVectorType& actualTensions);
    double findDesiredInternalTension(const double& fmin, const double& fmax, JointVectorType& jointTorques);
    void jointSpaceTorqueFeedback(const JointVectorType& torqueError, const JointVectorType& jointKp, const double& tensionError, const double& tensionKp, SliderVectorType& desiredSliders);
    
    void calcJacobian(const JointVectorType& jointPos, const JointVectorType& xy, const bool isLeft, JacobianType& jacobian);
    void jointToCartesian(const JointVectorType& jointPositions, const bool isLeft, JointVectorType& xy);
    void forceToJointTorque(const JacobianType& jacobian, const JointVectorType& force, JointVectorType& jointTorque);
    
private:
    ReferenceMatrixType                  referenceMatrix;
    ReferenceMatrixTransposeType         referenceMatrixTranspose;
    ReferenceMatrixTransposeInverseType  referenceMatrixTransposeInverse;
    SliderVectorType                     nullSpaceVector;
};

template<unsigned int N>
FingerKinematics<N>::FingerKinematics()
{
}

template<unsigned int N>
void FingerKinematics<N>::setReferenceMatrix(const ReferenceMatrixType &refMatrix_in)
{
    referenceMatrix = refMatrix_in;
    referenceMatrixTranspose = refMatrix_in.transpose();

    // svd
    VectorXd tmp(referenceMatrix.cols());
    VectorXd s(VectorXd::Zero(referenceMatrix.cols()));
    MatrixXd u(MatrixXd::Identity(referenceMatrix.rows(), referenceMatrix.cols()));
    MatrixXd v(MatrixXd::Identity(referenceMatrix.cols(), referenceMatrix.cols()));
    
    if (KDL::svd_eigen_HH(referenceMatrix, u, s, v, tmp, 100) < 0)
    {
        std::stringstream err;
        err << "SVD calculation failed" << std::endl;
        RCS::Logger::log("gov.nasa.controllers.FingerKinematics", log4cpp::Priority::ERROR, err.str());
        throw std::runtime_error(err.str());
        return;
    }
    
    double eps = 1.e-6;
    for (int i = 0; i < s.rows(); ++i)
    {
        if (s(i) > eps)
        {
            s(i) = 1.0 / s(i);
        }
        else
        {
            s(i) = 0;
        }
    }
    
    referenceMatrixTransposeInverse = (v * s.asDiagonal() * u.transpose()).transpose();
    
    nullSpaceVector = v.col(referenceMatrix.rows());
    if(nullSpaceVector[0] < 0)
		for (int i = 0; i < nullSpaceVector.rows(); ++i)
			nullSpaceVector[i] *= -1;
}

template<unsigned int N>
void FingerKinematics<N>::forward(const SliderVectorType& sliderPositions, JointVectorType& jointPositions)
{
    jointPositions = referenceMatrixTransposeInverse * sliderPositions;
}

template<unsigned int N>
void FingerKinematics<N>::inverse(const JointVectorType& jointPositions, SliderVectorType& sliderPositions)
{
    sliderPositions = referenceMatrixTranspose * jointPositions;
}

template<unsigned int N>
void FingerKinematics<N>::projectToRangeSpace(const SliderVectorType& sliderPositions_in, SliderVectorType& sliderPositions_out)
{
    sliderPositions_out = (referenceMatrixTranspose * referenceMatrixTransposeInverse) * sliderPositions_in;
}

//newly added
template<unsigned int N>
void FingerKinematics<N>::tensionToJointTorque(const SliderVectorType& actualTensions, JointVectorType& jointTorques)
{
    jointTorques = -referenceMatrix * actualTensions;
}

template<unsigned int N>
double FingerKinematics<N>::calculateInternalTension(const SliderVectorType& actualTensions)
{
    return nullSpaceVector.transpose() * actualTensions;
}

template<unsigned int N>
double FingerKinematics<N>::findDesiredInternalTension(const double& fmin, const double& fmax, JointVectorType& jointTorques)
{
    double t0 = 0;
    double d, alpha;
    int l, h;
    //find initial internal tension value
    for (int i = 0; i < jointTorques.rows()+1; ++i)
    {
        d = (fmin + (referenceMatrixTransposeInverse.col(i)).transpose() * jointTorques) / nullSpaceVector[i];
        if(d > t0)
            t0 = d;
    }
    
    //check to see if internal tensions are in range
    SliderVectorType fTmp = -referenceMatrixTransposeInverse.transpose()*jointTorques + nullSpaceVector*t0;
    if (fTmp.maxCoeff() <= fmax)
        return t0;

    //find index numbers for highest and lowest tensions
    h = -1; l = -1;
    for (int i = 0; i < fTmp.rows(); ++i)
    {
        if(fTmp(i) == fTmp.maxCoeff())
            h = i;
        if(fTmp(i) == fTmp.minCoeff())
            l = i;
    }
    if (h == -1 || l == -1)
    {
        RCS::Logger::log("gov.nasa.robonet.JointCommandFinger", log4cpp::Priority::WARN, "Index numbers l and h were not determined, check FingerKinematics.h");
        return t0;
    }

    //scale torques
    d = -(nullSpaceVector[h]*referenceMatrixTransposeInverse.col(l) - nullSpaceVector[l]*referenceMatrixTransposeInverse.col(h)).transpose() * jointTorques;
    alpha = (nullSpaceVector[h]*fmin - nullSpaceVector[l]*fmax) / d;
    t0 = -((fmax*referenceMatrixTransposeInverse.col(l) - fmin*referenceMatrixTransposeInverse.col(h)) / d).transpose() * jointTorques;
    jointTorques *= alpha;

    //Iterate if still out of range
    fTmp = -referenceMatrixTransposeInverse.transpose()*jointTorques + nullSpaceVector*t0;
    if (fTmp.maxCoeff() <= fmax)
        return t0;
    
    //find index numbers for highest and lowest tensions
    h = -1; l = -1;
    for (int i = 0; i < fTmp.rows(); ++i)
    {
        if(fTmp(i) == fTmp.maxCoeff())
            h = i;
        if(fTmp(i) == fTmp.minCoeff())
            l = i;
    }
    if (h == -1 || l == -1)
    {
        RCS::Logger::log("gov.nasa.robonet.JointCommandFinger", log4cpp::Priority::WARN, "Index numbers l and h were not determined, check FingerKinematics.h");
        return t0;
    }

    //scale torques
    d = -(nullSpaceVector[h]*referenceMatrixTransposeInverse.col(l) - nullSpaceVector[l]*referenceMatrixTransposeInverse.col(h)).transpose() * jointTorques;
    alpha = (nullSpaceVector[h]*fmin - nullSpaceVector[l]*fmax) / d;
    t0 = -((fmax*referenceMatrixTransposeInverse.col(l) - fmin*referenceMatrixTransposeInverse.col(h)) / d).transpose() * jointTorques;
    jointTorques *= alpha;
    return t0;
}

template<unsigned int N>
void FingerKinematics<N>::jointSpaceTorqueFeedback(const JointVectorType& torqueError, const JointVectorType& jointKp, const double& tensionError, const double& tensionKp, SliderVectorType& desiredSliders)
{
    desiredSliders = referenceMatrixTranspose * jointKp.asDiagonal() * torqueError - nullSpaceVector * tensionKp * tensionError;
}

//newly added for Cartesian
template<unsigned int N>
void FingerKinematics<N>::calcJacobian(const JointVectorType& jointPos, const JointVectorType& xy, const bool isLeft, JacobianType& jacobian)
{
    jacobian.setIdentity();
    if(jointPos.rows() == 3)
    {
        double L2, L3, r34;
        L2 = 30.48;
        L3 = 24.638;
        r34 = 7.0/12.0;
        //define Jacobian from proximal/medial joints to x-y frame
        jacobian(1,1) = -xy[2];
        jacobian(1,2) = -(L2*sin(jointPos[1]+jointPos[2]) + (1+r34)*L3*sin(jointPos[1]+(1+r34)*jointPos[2]));
        jacobian(2,1) = xy[1];
        jacobian(2,2) = (L2*cos(jointPos[1]+jointPos[2]) + (1+r34)*L3*cos(jointPos[1]+(1+r34)*jointPos[2]));
    }
    else if(jointPos.rows() == 4)
    {
        double L0 = 11.938;
        double L1 = 30.734;
        double L2 = 39.37;
        double L3 = 33.02;
        if(isLeft)
        {
            double alpha = -0.6981317;
            double phi = 1.309;
            double tempA = L2*cos(jointPos[2])+L3*cos(jointPos[2]+jointPos[3]);
            double tempB = L2*sin(jointPos[2])+L3*sin(jointPos[2]+jointPos[3]);
            double tempAA = L0 + (tempA+L1)*cos(jointPos[1]) - tempB*cos(alpha)*sin(jointPos[1]);
            double tempBB = (tempA+L1)*sin(jointPos[1]) + tempB*cos(alpha)*cos(jointPos[1]);
            jacobian(0,0) = -(cos(phi)*(tempB*cos(-jointPos[0])*sin(alpha) - tempAA*sin(-jointPos[0])));
            jacobian(0,1) = cos(jointPos[1])*(-tempB*cos(alpha)*cos(phi)*cos(-jointPos[0]) + (tempA+L1)*sin(phi)) - sin(jointPos[1])*((tempA+L1)*cos(phi)*cos(-jointPos[0]) + tempB*cos(alpha)*sin(phi));
            jacobian(0,2) = cos(phi)*(-tempB*cos(-jointPos[0])*cos(jointPos[1])+tempA*sin(alpha)*sin(-jointPos[0])) - tempB*sin(phi)*sin(jointPos[1]) + tempA*cos(alpha)*(cos(jointPos[1])*sin(phi)-cos(phi)*cos(-jointPos[0])*sin(jointPos[1]));
            jacobian(0,3) = 0;
            jacobian(1,0) = -(tempAA*cos(-jointPos[0])+tempB*sin(alpha)*sin(-jointPos[0]));
            jacobian(1,1) = -tempBB*sin(-jointPos[0]);
            jacobian(1,2) = -tempA*cos(-jointPos[0])*sin(alpha) - sin(-jointPos[0])*(tempB*cos(jointPos[1])+tempA*cos(alpha)*sin(jointPos[1]));
            jacobian(1,3) = 0;
            jacobian(2,0) = -(sin(phi)*(-tempB*cos(-jointPos[0])*sin(alpha) + tempAA*sin(-jointPos[0])));
            jacobian(2,1) = tempBB*cos(-jointPos[0])*sin(phi)+cos(phi)*((tempA+L1)*cos(jointPos[1])-tempB*cos(alpha)*sin(jointPos[1]));
            jacobian(2,2) = tempB*cos(-jointPos[0])*cos(jointPos[1])*sin(phi)-tempA*sin(alpha)*sin(phi)*sin(-jointPos[0])-tempB*cos(phi)*sin(jointPos[1])+tempA*cos(alpha)*(cos(phi)*cos(jointPos[1])+cos(-jointPos[0])*sin(phi)*sin(jointPos[1]));
            jacobian(2,3) = 0;
        }
        else
        {
            double alpha = 0.6981317;
            double phi = -1.309;
            double tempA = L2*cos(jointPos[2])+L3*cos(jointPos[2]+jointPos[3]);
            double tempB = L2*sin(jointPos[2])+L3*sin(jointPos[2]+jointPos[3]);
            double tempAA = L0 + (tempA+L1)*cos(jointPos[1]) - tempB*cos(alpha)*sin(jointPos[1]);
            double tempBB = (tempA+L1)*sin(jointPos[1]) + tempB*cos(alpha)*cos(jointPos[1]);
            jacobian(0,0) = cos(phi)*(tempB*cos(jointPos[0])*sin(alpha) - tempAA*sin(jointPos[0]));
            jacobian(0,1) = cos(jointPos[1])*(-tempB*cos(alpha)*cos(phi)*cos(jointPos[0]) + (tempA+L1)*sin(phi)) - sin(jointPos[1])*((tempA+L1)*cos(phi)*cos(jointPos[0]) + tempB*cos(alpha)*sin(phi));
            jacobian(0,2) = cos(phi)*(-tempB*cos(jointPos[0])*cos(jointPos[1])+tempA*sin(alpha)*sin(jointPos[0])) - tempB*sin(phi)*sin(jointPos[1]) + tempA*cos(alpha)*(cos(jointPos[1])*sin(phi)-cos(phi)*cos(jointPos[0])*sin(jointPos[1]));
            jacobian(0,3) = 0;
            jacobian(1,0) = tempAA*cos(jointPos[0])+tempB*sin(alpha)*sin(jointPos[0]);
            jacobian(1,1) = -tempBB*sin(jointPos[0]);
            jacobian(1,2) = -tempA*cos(jointPos[0])*sin(alpha) - sin(jointPos[0])*(tempB*cos(jointPos[1])+tempA*cos(alpha)*sin(jointPos[1]));
            jacobian(1,3) = 0;
            jacobian(2,0) = sin(phi)*(-tempB*cos(jointPos[0])*sin(alpha) + tempAA*sin(jointPos[0]));
            jacobian(2,1) = tempBB*cos(jointPos[0])*sin(phi)+cos(phi)*((tempA+L1)*cos(jointPos[1])-tempB*cos(alpha)*sin(jointPos[1]));
            jacobian(2,2) = tempB*cos(jointPos[0])*cos(jointPos[1])*sin(phi)-tempA*sin(alpha)*sin(phi)*sin(jointPos[0])-tempB*cos(phi)*sin(jointPos[1])+tempA*cos(alpha)*(cos(phi)*cos(jointPos[1])+cos(jointPos[0])*sin(phi)*sin(jointPos[1]));
            jacobian(2,3) = 0;
        }
    }
}

template<unsigned int N>
void FingerKinematics<N>::jointToCartesian(const JointVectorType& jointPos, const bool isLeft, JointVectorType& xy)
{
    if(jointPos.rows() == 3)
    {
        xy[0] = jointPos[0];
        double L1 = 44.45;
        double L2 = 30.48;
        double L3 = 24.638;
        double r34 = 7.0/12.0;
        xy[1] = L1*cos(jointPos[1]) + L2*cos(jointPos[1]+jointPos[2]) + L3*cos(jointPos[1]+(1+r34)*jointPos[2]);
        xy[2] = L1*sin(jointPos[1]) + L2*sin(jointPos[1]+jointPos[2]) + L3*sin(jointPos[1]+(1+r34)*jointPos[2]);
    }
    else if(jointPos.rows() == 4)
    {
        double L0 = 11.938;
        double L1 = 30.734;
        double L2 = 39.37;
        double L3 = 33.02;
        if(isLeft)
        {
            double alpha = -0.6981317;
            double phi = 1.309;
            double tempA = L1+L2*cos(jointPos[2])+L3*cos(jointPos[2]+jointPos[3]);
            double tempB = L2*sin(jointPos[2])+L3*sin(jointPos[2]+jointPos[3]);
		    xy[0] = sin(phi)*(tempA*sin(jointPos[1]) + cos(alpha)*cos(jointPos[1])*tempB) + cos(phi)*(sin(alpha)*sin(-jointPos[0])*tempB + cos(-jointPos[0])*(L0+cos(jointPos[1])*tempA - cos(alpha)*sin(jointPos[1])*tempB));
		    xy[1] = -cos(-jointPos[0])*sin(alpha)*tempB + sin(-jointPos[0])*(L0+cos(jointPos[1])*tempA-cos(alpha)*sin(jointPos[1])*tempB);
		    xy[2] = sin(phi)*(-sin(alpha)*sin(-jointPos[0])*tempB - cos(-jointPos[0])*(L0+cos(jointPos[1])*tempA-cos(alpha)*sin(jointPos[1])*tempB)) + cos(phi)*(sin(jointPos[1])*tempA + cos(alpha)*cos(jointPos[1])*tempB);
            xy[3] = jointPos[3];
        }
        else
        {
            double alpha = 0.6981317;
            double phi = 1.309;
            double tempA = L1+L2*cos(jointPos[2])+L3*cos(jointPos[2]+jointPos[3]);
            double tempB = L2*sin(jointPos[2])+L3*sin(jointPos[2]+jointPos[3]);
		    xy[0] = sin(phi)*(tempA*sin(jointPos[1]) + cos(alpha)*cos(jointPos[1])*tempB) + cos(phi)*(sin(alpha)*sin(-jointPos[0])*tempB + cos(-jointPos[0])*(L0+cos(jointPos[1])*tempA - cos(alpha)*sin(jointPos[1])*tempB));
		    xy[1] = -cos(-jointPos[0])*sin(alpha)*tempB + sin(-jointPos[0])*(L0+cos(jointPos[1])*tempA-cos(alpha)*sin(jointPos[1])*tempB);
		    xy[2] = sin(phi)*(-sin(alpha)*sin(-jointPos[0])*tempB - cos(-jointPos[0])*(L0+cos(jointPos[1])*tempA-cos(alpha)*sin(jointPos[1])*tempB)) + cos(phi)*(sin(jointPos[1])*tempA + cos(alpha)*cos(jointPos[1])*tempB);
            xy[3] = jointPos[3];
        }
	}
//	else
//        xy = jointPos;
}

template<unsigned int N>
void FingerKinematics<N>::forceToJointTorque(const JacobianType& jacobian, const JointVectorType& force, JointVectorType& jointTorque)
{
    if(jointTorque.rows() >= 3)
    {
        //multiply by J^T
        jointTorque = jacobian.transpose() * force;
    }
//    else
//        jointTorque = force;
}

#endif
