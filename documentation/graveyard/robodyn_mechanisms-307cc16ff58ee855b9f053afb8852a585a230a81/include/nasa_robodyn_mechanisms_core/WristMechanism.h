/*!
 * @file  WristMechanism.h
 * @brief Calculates the wrist fwd and inv kinematics as well as limits
 * @author Allison Thackston
 * @date 3/22/2013
 */

#ifndef WRIST_MECHANISM_H
#define WRIST_MECHANISM_H

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/LU>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Eigenvalues>
#include <vector>

/***************************************************************************//**
 *
 * @brief This class provides the utilities for wrist operation including
 * kinematics and calibration
 *
 ******************************************************************************/
class WristMechanism{
public:

    WristMechanism();
    virtual ~WristMechanism();

    void outputLimits();
    void setSliderOffsetGain(Eigen::Vector2f sliderOffset, Eigen::Vector2f sliderGain);
    void outputSliderOffsetGain();
    void setAngleOffsetGain(Eigen::Vector2f angleOffset, Eigen::Vector2f angleGain);
    void outputAngleOffsetGain();
    void loadDesignParams(Eigen::Vector3f A, Eigen::Vector3f A0, Eigen::Vector3f B, Eigen::Vector3f C, Eigen::Vector3f D, Eigen::Vector3f D0, Eigen::Vector3f Pitch, Eigen::Vector3f Yaw, Eigen::Vector3f M, Eigen::Vector3f N, float linkLength);
    void outputDesignParams();
    void applyLimits(float &pitch, float &yaw);
    void getLimits(float pitch, float yaw, float &upperPitch, float &lowerPitch, float &upperYaw, float &lowerYaw);
    void setLimits(float upperPitchLim, float lowerPitchLim, float upperYawLim, float lowerYawLim);
    void setLimitOffsetGain(float upperPitchOffset, float upperPitchGain, float lowerPitchOffset, float lowerPitchGain, float upperYawOffset, float upperYawGain, float lowerYawOffset, float lowerYawGain);


    Eigen::Vector2f getAngleFromSlider(Eigen::Vector2f slider);
    Eigen::Vector2f getSliderFromAngle(Eigen::Vector2f ang);

    Eigen::Vector2f getWristEncoderFromSlider(Eigen::Vector2f slider);
    Eigen::Vector2f getSliderFromWristEncoder(Eigen::Vector2f encoder);

    void tareWristEncoders(Eigen::Vector2f encoder);
    void calWrist(Eigen::Vector2f encoder, Eigen::Vector2f calSliderPos, Eigen::Vector2f angle);

    /// @brief true if calWrist has been called and successfully completed
    bool isCalibrated;

    Eigen::Vector2f NewtonsMethod(Eigen::Vector2f ang, Eigen::Vector2f pos);
    /// @brief the maximum number of iterations to perform NewtonsMethod
    int maxIt;
    /// @brief the maximum difference between NewtonsMethod solutions before stopping
    float eps;


protected:

    float upperPitchLim;
    float lowerPitchLim;
    float upperYawLim;
    float lowerYawLim;

    float upperPitchOffset;
    float upperPitchGain;
    float lowerPitchOffset;
    float lowerPitchGain;
    float upperYawOffset;
    float upperYawGain;
    float lowerYawOffset;
    float lowerYawGain;

    Eigen::Vector2f sliderOffset;
    Eigen::Vector2f sliderGain;
    Eigen::Vector2f angleOffset;
    Eigen::Vector2f angleGain;

    float linkLength;
    Eigen::Vector3f alpha,beta,gamma,delta,m,n,u,v,p;

    float lengthSq;

    Eigen::Matrix3f R1m, R1n, Rsm, Rsn, Rcm, Rcn;

    Eigen::Vector2f encoderTarePos;
    Eigen::Vector2f calSliderPos;
    Eigen::Vector2f lastAng;

    enum RotationType { ONE, SIN, COS };

    Eigen::Matrix3f AA2Rot(Eigen::Vector3f axis, RotationType index);
    Eigen::Matrix3f AA2Rot(Eigen::Vector3f axis, float theta);

    int doFwdKin(Eigen::Vector2f &ang, Eigen::Vector2f pos, Eigen::Vector2f lastang);

    int doInvKin(Eigen::Vector2f &pos, Eigen::Vector2f ang);

    struct Partials
    {
        float f0;
        float f;
        float fq;
        float ftheta;
        float fphi;
    };

    Partials findPartialDerivatives(Eigen::Vector3f a, Eigen::Vector3f b, Eigen::Vector3f u, float q, Eigen::Vector2f ang);

    Eigen::Matrix3f findFwdKinMatrix(Eigen::Vector3f a, Eigen::Vector3f b, Eigen::Vector3f u, float q);

};

#endif // WRIST_MECHANISM_H
