/**
  * @file WristMechanism.cpp
  *
  * @brief WristMechanism includes inverse and forward kinematics, as well as calibration functions for the wrist
  * @note The values in a0 in old code also did not match the values in slides.  If there
  *       are issues with the new inverse kinematics, try changing the vector or the param file
  * @note I also changed fwd kinematics from a lookup table to computation.  If fwd kinematics
  *       proves to be slow, try reverting to a lookup table.
  **/

#include "nasa_robodyn_mechanisms_core/WristMechanism.h"
#include "nasa_common_utilities/Logger.h"
#include <Eigen/LU>

using namespace Eigen;
using namespace std;
using namespace log4cpp;

WristMechanism::WristMechanism()
{
    maxIt = 20;
    eps = 0.001;
    isCalibrated=false;
}

WristMechanism::~WristMechanism()
{
}

/**
  * @brief sets the slider offset and gain for calculating encoder-slider relationship
  * @param sliderOffset the offset of the slider
  * @param sliderGain the gain of the slider
  **/
void WristMechanism::setSliderOffsetGain(Vector2f sliderOffset, Vector2f sliderGain)
{
    this->sliderOffset = sliderOffset;
    this->sliderGain = sliderGain;
}

void WristMechanism::outputSliderOffsetGain()
{
    std::cout<<"Slider Offset: "<<std::endl<<this->sliderOffset<<std::endl;
    std::cout<<"Slider Gain: "<<std::endl<<this->sliderGain<<std::endl;
}

/**
  * @brief sets the angle offset and gain when calculating getAngleFromWristEncoder
  * @param angleOffset the offset of the angle
  * @param angelGain the gain of the angle
  **/
void WristMechanism::setAngleOffsetGain(Vector2f angleOffset, Vector2f angleGain)
{
    this->angleOffset = angleOffset;
    this->angleGain = angleGain;
}

void WristMechanism::outputAngleOffsetGain()
{
    std::cout<<"Angle Offset: "<<std::endl<<this->angleOffset<<std::endl;
    std::cout<<"Angle Gain: "<<std::endl<<this->angleGain<<std::endl;
}

/**
  * @brief loads the physical design parameters of the wrist
  * @note here might be a good place to put in vectors that become negative because they are left vs right
  * @param A outer arm forward position
  * @param A0 origin of A (or point along axis of A) -- known point for calibration
  * @param B outer wrist position
  * @param C inner wrist position
  * @param D inner forward arm forward position
  * @param D0 origin of D (or point along axis of D) -- known point for calibration
  * @param Pitch origin of pitch axis
  * @param Yaw origin of yaw axis
  * @param PalmCenter center point of palm
  * @param M axis about which the pitch rotates (most likely y, unless frames have changed)
  * @param N axis about which the yaw rotates (most likey x, unless frames have changed)
  * @param maxTravel the amount of distance an actuator can travel (in mm)
  * @param linkLength the length of a link (A-B) or (C-D)
  * @throws runtime_error if A - A0 has no normal
  * @throws runtime_error if D - D0 has no normal
  **/
void WristMechanism::loadDesignParams(Vector3f A, Vector3f A0, Vector3f B, Vector3f C, Vector3f D, Vector3f D0, Vector3f Pitch, Vector3f Yaw, Vector3f M, Vector3f N, float linkLength)
{
    this->linkLength = linkLength;

    //shift origin to pitch center
    alpha = A0 - Pitch;
    beta = B - Yaw;
    gamma = C - Yaw;
    delta = D0 - Pitch;
    p = Yaw - Pitch;

    // find unit vector for 1st slider axis
    Vector3f uMag = A - A0;
    float uNorm = uMag.norm();
    if(uNorm!=0)
    {
        u =uMag/uNorm;
    }
    else
    {
        std::stringstream err;
        err << "in loadDesignParams A and A0 cannot be the same or produce a normal of 0";
        RCS::Logger::log("gov.nasa.robonet.WristMechanism", log4cpp::Priority::ERROR, err.str());
        throw std::runtime_error(err.str());
    }

    // find unit vector for 2nd slider axis
    Vector3f vMag = D - D0;
    float vNorm = vMag.norm();
    if(vNorm!=0)
    {
        v = vMag/vNorm;
    }
    else
    {
        std::stringstream err;
        err << "in loadDesignParams D and D0 cannot be the same or produce a normal of 0";
        RCS::Logger::log("gov.nasa.robonet.WristMechanism", log4cpp::Priority::ERROR, err.str());
        throw std::runtime_error(err.str());
    }

    // find length^2
    lengthSq = linkLength * linkLength;

    m = M;
    n = N;

    R1m = AA2Rot(m, ONE);
    R1n = AA2Rot(n, ONE);
    Rsm = AA2Rot(m, SIN);
    Rsn = AA2Rot(n, SIN);
    Rcm = AA2Rot(m, COS);
    Rcn = AA2Rot(n, COS);

}

void WristMechanism::outputDesignParams()
{
    std::cout<<"a:"<<std::endl<<alpha<<std::endl;
    std::cout<<"b: "<<std::endl<<beta<<std::endl;
    std::cout<<"C: "<<std::endl<<gamma<<std::endl;
    std::cout<<"d: "<<std::endl<<delta<<std::endl;

    std::cout<<"m: "<<std::endl<<m<<std::endl;
    std::cout<<"n: "<<std::endl<<n<<std::endl;
    std::cout<<"u: "<<std::endl<<u<<std::endl;
    std::cout<<"v: "<<std::endl<<v<<std::endl;
    std::cout<<"p: "<<std::endl<<p<<std::endl;
    std::cout<<"Link Length: "<<linkLength<<std::endl;
}

void WristMechanism::setLimitOffsetGain(float upperPitchOffset, float upperPitchGain, float lowerPitchOffset, float lowerPitchGain, float upperYawOffset, float upperYawGain, float lowerYawOffset, float lowerYawGain)
{
    this->upperPitchOffset = upperPitchOffset;
    this->upperPitchGain = upperPitchGain;
    this->lowerPitchOffset = lowerPitchOffset;
    this->lowerPitchGain = lowerPitchGain;
    this->upperYawOffset = upperYawOffset;
    this->upperYawGain = upperYawGain;
    this->lowerYawOffset = lowerYawOffset;
    this->lowerYawGain = lowerYawGain;

}

/**
  * @brief sets the overall limits of the task space
  * @param upperPitchLim upper limit of pitch
  * @param lowerPitchLim lower limit of pitch
  * @param upperYawLim lower limit of yaw
  * @param lowerYawLim lower limit of yaw
  **/
void WristMechanism::setLimits(float upperPitchLim, float lowerPitchLim, float upperYawLim, float lowerYawLim)
{
    this->upperPitchLim = upperPitchLim;
    this->lowerPitchLim = lowerPitchLim;

    this->upperYawLim = upperYawLim;
    this->lowerYawLim = lowerYawLim;

}

/**
  * @brief limits the pitch and yaw to valid values (based on formula of task space)
  * @param pitch
  * @param yaw
  **/
void WristMechanism::applyLimits(float &pitch, float &yaw)
{

    float up, lp, uy, ly;
    getLimits(pitch, yaw, up, lp, uy, ly );

    if(pitch < lp)
    {
        RCS::Logger::getCategory("gov.nasa.robonet.WristMechanism")<<log4cpp::Priority::DEBUG<<"limiting lower pitch " << pitch <<" to "<< lp;
        pitch = lp;
    }
    if(pitch > up)
    {
        RCS::Logger::getCategory("gov.nasa.robonet.WristMechanism")<<log4cpp::Priority::DEBUG<<"limiting upper pitch " << pitch <<" to "<< up;
        pitch = up;
    }
    if(yaw < ly)
    {
        RCS::Logger::getCategory("gov.nasa.robonet.WristMechanism")<<log4cpp::Priority::DEBUG<<"limiting lower yaw " << yaw <<" to "<< ly;
        yaw = ly;
    }
    if(yaw > uy)
    {
        RCS::Logger::getCategory("gov.nasa.robonet.WristMechanism")<<log4cpp::Priority::DEBUG<<"limiting upper yaw " << yaw<<" to "<< uy;
        yaw = uy;
    }
}


/**
  * @brief gets the limits on pitch and yaw based on current pitch and yaw
  * @note right now, only upper pitch is variable
  * @param pitch
  * @param yaw
  * @param outputs upperPitch
  * @param outputs upperYaw
  * @param outputs lowerPitch
  * @param outputs lowerYaw
  **/
void WristMechanism::getLimits(float pitch, float yaw, float &upperPitch, float &lowerPitch, float &upperYaw, float &lowerYaw)
{
    Vector2f upperPitchLims;

    upperPitchLims(0) = upperPitchLim;
    upperPitchLims(1) = upperPitchGain * yaw + upperPitchOffset;

    upperPitch = upperPitchLims.minCoeff();

    Vector2f lowerPitchLims;

    lowerPitchLims(0) = lowerPitchLim;
    lowerPitchLims(1) = lowerPitchGain * yaw + lowerPitchOffset;

    lowerPitch = lowerPitchLims.maxCoeff();

    Vector2f upperYawLims;
    upperYawLims(0) = upperYawLim;
    upperYawLims(1) = upperYawGain*pitch + upperYawOffset;

    upperYaw = upperYawLims.minCoeff();

    Vector2f lowerYawLims;

    lowerYawLims(0) = lowerYawLim;
    lowerYawLims(1) = lowerYawGain * pitch + lowerYawOffset;

    lowerYaw = lowerYawLims.maxCoeff();
}

void WristMechanism::outputLimits()
{
    std::cout<<"Upper Pitch: "<<std::endl<<this->upperPitchLim<<std::endl;
    std::cout<<"Lower Pitch: "<<std::endl<<this->lowerPitchLim<<std::endl;

    std::cout<<"Upper Yaw: "<<std::endl<<this->upperYawLim<<std::endl;
    std::cout<<"Lower Yaw: "<<std::endl<<this->lowerYawLim<<std::endl;

    std::cout<<"Upper Pitch Offset: "<<this->upperPitchOffset<<std::endl;
    std::cout<<"Upper Pitch Gain: "<<this->upperPitchGain<<std::endl;
    std::cout<<"Upper Yaw Offset: "<<this->upperYawOffset<<std::endl;
    std::cout<<"Upper Yaw Gain: "<<this->upperYawGain<<std::endl;

}

/**
  * @brief calculates the forward kinematics
  * @param ang the output angle (pitch yaw) of wrist
  * @param pos the input slider position
  * @param lastang the previous angle (final step on reducing solution)
  * @returns 0 for success
  **/
int WristMechanism::doFwdKin(Vector2f &ang, Vector2f pos, Vector2f lastang)
{
    lastang(0) = (lastang(0)-angleOffset(0))/angleGain(0);
    lastang(1) = (lastang(1)-angleOffset(1))/angleGain(1);

    ang = Vector2f::Zero();

    Matrix3f AB = findFwdKinMatrix(alpha, beta, this->u, pos(0));
    Matrix3f DC = findFwdKinMatrix(delta, gamma, this->v, pos(1));

    MatrixXf Cfk = MatrixXf::Zero(8,8);
    MatrixXf Dfk = MatrixXf::Zero(8,8);
    MatrixXf Efk = MatrixXf::Zero(8,8);
    MatrixXf Ffk = MatrixXf::Zero(8,8);


    Cfk <<  AB(0,0), AB(0,1), AB(0,2), 0,       AB(1,0), AB(1,1), AB(2,0), AB(2,1),
            0,       0,       0,       0,       AB(0,0), AB(0,1), AB(1,0), AB(1,1),
            0,       AB(0,0), AB(0,1), AB(0,2), 0,       AB(1,0), 0,       AB(2,0),
            0,       0,       0,       0,       0,       AB(0,0), 0,       AB(1,0),
            DC(0,0), DC(0,1), DC(0,2), 0,       DC(1,0), DC(1,1), DC(2,0), DC(2,1),
            0,       0,       0,       0,       DC(0,0), DC(0,1), DC(1,0), DC(1,1),
            0,       DC(0,0), DC(0,1), DC(0,2), 0,       DC(1,0), 0,       DC(2,0),
            0,       0,       0,       0,       0,       DC(0,0), 0,       DC(1,0);



    Dfk <<  AB(1,2), 0,       AB(2,2), 0,       0,       0,       0,       0,
            AB(0,2), 0,       AB(1,2), 0,       AB(2,0), AB(2,1), AB(2,2), 0,
            AB(1,1), AB(1,2), AB(2,1), AB(2,2), 0,       0,       0,       0,
            AB(0,1), AB(0,2), AB(1,1), AB(1,2), 0,       AB(2,0), AB(2,1), AB(2,2),
            DC(1,2), 0,       DC(2,2), 0,       0,       0,       0,       0,
            DC(0,2), 0,       DC(1,2), 0,       DC(2,0), DC(2,1), DC(2,2), 0,
            DC(1,1), DC(1,2), DC(2,1), DC(2,2), 0,       0,       0,       0,
            DC(0,1), DC(0,2), DC(1,1), DC(1,2), 0,       DC(2,0), DC(2,1), DC(2,2);


    Efk <<  0, 0, 0, 0, 1, 0, 0, 0,
            0, 0, 0, 0, 0, 1, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 1, 0,
            0, 0, 0, 0, 0, 0, 0, 1,
            0, 0, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 0;

    Ffk <<  0, 0, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 0,
            1, 0, 0, 0, 0, 0, 0, 0,
            0, 1, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 1, 0, 0, 0,
            0, 0, 0, 0, 0, 1, 0, 0;


    MatrixXf Dfki = Dfk.lu().inverse();

    MatrixXf Mfk = (Efk - (Ffk*Dfki*Cfk));

    EigenSolver<MatrixXf> es(Mfk);

    // the eigenvalues are solutions for x
    VectorXcf xtheta = es.eigenvalues();

    // the eigenvectors give y as y= m2'' / m1''
    MatrixXcf ephi = es.eigenvectors();

    VectorXcf yphi = VectorXcf::Zero(8);

    if(ephi.rows()!=8)
    {
        std::stringstream err;
        err << "in doFwdKin there should be 8 rows in ephi";
        RCS::Logger::log("gov.nasa.robonet.WristMechanism", log4cpp::Priority::ERROR, err.str());
        throw std::runtime_error(err.str());
    }
    if(ephi.cols()!=8)
    {
        std::stringstream err;
        err << "in doFwdKin there should be 8 cols in ephi";
        RCS::Logger::log("gov.nasa.robonet.WristMechanism", log4cpp::Priority::ERROR, err.str());
        throw std::runtime_error(err.str());
    }
    if(yphi.rows()!=8)
    {
        std::stringstream err;
        err << "in doFwdKin there should be 8 rows in yphi";
        RCS::Logger::log("gov.nasa.robonet.WristMechanism", log4cpp::Priority::ERROR, err.str());
        throw std::runtime_error(err.str());
    }

    std::complex<float> zero(0,0);
    for(unsigned int i = 0; i< ephi.rows(); i++)
    {
        if(ephi(0,i)!=zero){
            yphi(i) = ephi(1,i)/ephi(0,i);
        }
        else
        {
            std::stringstream err;
            err << "in doFwdKin -- divide by zero error in calculating yphi for "<< ephi(i,1)<<", M1'' is zero " << ephi(i,0);
            RCS::Logger::log("gov.nasa.robonet.WristMechanism", log4cpp::Priority::ERROR, err.str());
            throw std::runtime_error(err.str());
        }

    }

    std::vector<float> ptheta;
    std::vector<float> pphi;
    double tantheta, tanphi, sintheta, costheta, theta, sinphi, cosphi, phi, dist;

    ptheta.clear();
    pphi.clear();
    // remove complex solutions
    for(unsigned int i = 0; i<xtheta.rows(); i++)
    {
        if(xtheta(i).imag()==0 && yphi(i).imag() ==0)
        {
            //solve for theta and phi
            tantheta = xtheta(i).real();
            tanphi = yphi(i).real();

            sintheta = (2*tantheta)/(1+tantheta*tantheta);
            costheta = (1-tantheta*tantheta)/(1+tantheta*tantheta);
            theta = atan2(sintheta, costheta);

            sinphi = 2*tanphi/(1+tanphi*tanphi);
            cosphi = (1-tanphi*tanphi)/(1+tanphi*tanphi);
            phi = atan2(sinphi, cosphi);

            //remove solutions outside of joint space for manipulator (within 5 degrees of limits to account for accumulated error)
            if(theta > lowerPitchLim - .1 && theta < upperPitchLim + .1 &&
                    phi > lowerYawLim - .1 && phi < upperYawLim + .1)
            {
                //std::cout<<"added (" <<theta <<","<<phi<<")"<<std::endl;
                ptheta.push_back(theta);
                pphi.push_back(phi);
            }


        }
    }

    if(ptheta.size() != pphi.size())
    {
        std::stringstream err;
        err << "in doFwdKin vectors ptheta and pphi should be the same size";
        RCS::Logger::log("gov.nasa.robonet.WristMechanism", log4cpp::Priority::ERROR, err.str());
        return -1;
    }


    // choose solution nearest to previous solution
    if(ptheta.size()>1 && pphi.size()>1)
    {
//        stringstream ss;
//        ss << "there is still more than one solution to fwd kinematics.. "<<std::endl;
//        ss <<"choose nearest to lastpos: ("<<lastang(0)<<","<<lastang(1)<<")"<<std::endl;
//        for(unsigned int k = 0; k<ptheta.size(); k++)
//        {
//            ss <<"("<<ptheta.at(k)<<","<<pphi.at(k)<<")"<<std::endl;
//        }
//        RCS::Logger::log("gov.nasa.robonet.JointCommandWrist", Priority::DEBUG, ss.str());
//        std::cout << ss.str();

        unsigned int i = 0;

        theta = ptheta.at(i);
        phi = pphi.at(i);

        dist = sqrt(pow(theta-lastang(0), 2.0)+ pow(phi-lastang(1), 2.0));

        ang(0) = theta;
        ang(1) = phi;

        i++;
        for(; i<ptheta.size(); i++)
        {
            theta = ptheta.at(i);
            phi = pphi.at(i);

            if(dist > sqrt(pow((theta-lastang(0)), 2.0)+ pow((phi-lastang(1)), 2.0)))
            {
                dist = sqrt(pow((theta-lastang(0)), 2.0)+ pow((phi-lastang(1)), 2.0));

                ang(0) = theta;
                ang(1) = phi;
            }
        }
    }
    else if(ptheta.size()==1 && pphi.size()==1)
    {

        ang(0) = ptheta.at(0);
        ang(1) = pphi.at(0);
    }
    else
    {
        return -1;
    }

//    //do newton's method
//    Vector2f dang = NewtonsMethod(ang, pos);
//    int maxit = maxIt;
//    while(maxit > 0 && (fabs(dang(0)) > eps || fabs(dang(1)) > eps))
//    {
//        ang = ang-dang;
//        dang = NewtonsMethod(ang, pos);
//        maxit--;

//        //std::cout<<"iterating"<<std::endl;
//    }

//    std::cout<<"iterations: "<<maxIt-maxit<<std::endl;

//    if(abs(dang(0))> eps || abs(dang(1)) > eps)
//    {
//        std::stringstream err;
//        err << "Fwd Kinematics solution not good"<<std::endl;
//        err << "ang: "<<std::endl<<ang<<std::endl;
//        err << "dang: "<<std::endl<<dang<<std::endl;
//        RCS::Logger::log("gov.nasa.robonet.WristMechanism", log4cpp::Priority::WARN, err.str());
//        return -1;
//    }

    return 0;

}

Matrix3f WristMechanism::findFwdKinMatrix(Vector3f a, Vector3f b, Vector3f u, float q)
{
    Matrix3f aOut;

    Vector3f ahat = a+q*u;
    RowVector3f pT = p.transpose();
    RowVector3f ahatT = ahat.transpose();

    float a1, b1, c1, d1, e1, f1, g1, h1, k1;

    a1 = p.dot(p) + b.dot(b) + ahat.dot(ahat) - lengthSq + 2*pT*R1n*b - 2*ahatT*R1m*(p+R1n*b);
    float b11 = -2*ahatT*R1m*Rsn*b;
    b1 = b11 + 2*pT*Rsn*b ;
    float c11 = -2*ahatT*R1m*Rcn*b;
    c1 = c11 + 2*pT*Rcn*b ;
    d1 = -2*ahatT*Rsm*(p+R1n*b);
    e1 = -2*ahatT*Rsm*Rsn*b;
    f1 = -2*ahatT*Rsm*Rcn*b;
    g1 = -2*ahatT*Rcm*(p+R1n*b);
    h1 = -2*ahatT*Rcm*Rsn*b;
    k1 = -2*ahatT*Rcm*Rcn*b;


    aOut(0,0) = a1+c1+g1+k1;
    aOut(1,0) = 2*d1 + 2*f1;
    aOut(2,0) = a1+c1-g1-k1;

    aOut(0,1) = 2*b1+2*h1;
    aOut(1,1) = 4*e1;
    aOut(2,1) = 2*b1-2*h1;

    aOut(0,2) = a1-c1+g1-k1;
    aOut(1,2) = 2*d1 - 2*f1;
    aOut(2,2) = a1-c1-g1+k1;

    return aOut;
}

/**
  * @brief calculates the inverse kinematics
  * @param ang the input angle (pitch yaw) of wrist
  * @param pos the output slider position
  * @returns 0 for success
  **/
int WristMechanism::doInvKin(Vector2f &pos, Vector2f ang)
{
    int success = 0;
    float pitch = ang(0);
    float yaw = ang(1);

    //applyLimits(pitch, yaw);

    float bravo, charlie, david;
    Vector3f r1, r2;

    // find rotation matrices around the pitch axis
    Matrix3f Rpitch = AA2Rot(m, pitch);
    // find rotation matrices around the yaw axis
    Matrix3f Ryaw = AA2Rot(n, yaw);


    r1 = Ryaw*beta;
    r2 = Rpitch*(p+r1);

    bravo = -2*u.dot(r2) + 2*alpha.dot(u);
    charlie = p.dot(p) + 2* p.dot(r1) + beta.dot(beta) - 2*alpha.dot(r2) + alpha.dot(alpha) - lengthSq;
    david = bravo*bravo - 4*charlie;

    if(david>=0)
    {
        pos(0) = -0.5*bravo - 0.5*sqrt(david);
    }
    else
    {
        success = -1;
    }

    r1 = Ryaw*gamma;
    r2 = Rpitch*(p+r1);

    bravo = -2*v.dot(r2) + 2*delta.dot(v);
    charlie = p.dot(p) + 2*p.dot(r1) + gamma.dot(gamma) -2*delta.dot(r2) + delta.dot(delta) - lengthSq;
    david = bravo*bravo - 4*charlie;
    if(david>=0)
    {
         pos(1) = -0.5*bravo - 0.5*sqrt(david);
    }
    else
    {
        success = -1;
    }

    return success;
}


/**
  * @brief gets the current angle (pitch and yaw) given the current slider pos (in mm) using fwd kinematics
  * @param slider the current slider value
  * @returns the current angle (pitch and yaw)
  * @throws runtime_error if fwd kinematic solution does not converge
  **/
Vector2f WristMechanism::getAngleFromSlider(Vector2f slider)
{
    Vector2f ang;
    if(doFwdKin(ang, slider, lastAng)!=0)
    {
        std::stringstream err;
        err << "Could not find fwd kinematic solution";
        throw std::runtime_error(err.str());
    }
    //std::cout<<"angle out of fwd kin: "<<std::endl<<ang<<std::endl;
    ang(0) = ang(0) * angleGain(0) + angleOffset(0);
    ang(1) = ang(1) * angleGain(1) + angleOffset(1);
    lastAng = ang;

    return ang;
}

/**
  * @brief gets the slider value from the current encoder value
  * @param encoder the current encoder value
  * @returns the current slider position (in mm)
  **/
Vector2f WristMechanism::getSliderFromWristEncoder(Vector2f encoder)
{
    if(!isCalibrated)
        throw std::runtime_error("wrist has not been calibrated");

    Vector2f encoderTared = encoder - encoderTarePos;

    Vector2f taredSliderEncoderPos = encoderTared + calSliderPos;

    Vector2f sliderPos;

    sliderPos(0) = taredSliderEncoderPos(0)*sliderGain(0) + sliderOffset(0);
    sliderPos(1) = taredSliderEncoderPos(1)*sliderGain(1) + sliderOffset(1);

    return sliderPos;
}

/**
  * @brief gets the encoder value from the current slider position
  * @param slider the current slider position (in mm)
  * @returns the current encoder value;
  **/
Vector2f WristMechanism::getWristEncoderFromSlider(Vector2f sliderPos)
{
    if(!isCalibrated)
        throw std::runtime_error("wrist has not been calibrated");
    Vector2f encoder, sliderEncoderPos, encoderTared;

    sliderEncoderPos(0) = (sliderPos(0) - sliderOffset(0))/sliderGain(0);
    sliderEncoderPos(1) = (sliderPos(1) - sliderOffset(1))/sliderGain(1);

    encoderTared = sliderEncoderPos - calSliderPos;
    encoder = encoderTared + encoderTarePos;

    return encoder;
}

/**
  * @brief gets the slider position from the current angle using inv kinematics
  * @param ang the current angle of the wrist (either from halls or fwd kinematics)
  * @returns the current slider position (in mm)
  * @throws runtime_error if inv kinematic solution not solvable
  **/
Vector2f WristMechanism::getSliderFromAngle(Vector2f ang)
{
    Vector2f angle;
    angle(0) = (ang(0) - angleOffset(0))/angleGain(0);
    angle(1) = (ang(1) - angleOffset(1))/angleGain(1);
    //angle = ang;

    //std::cout<<"sending in angle: "<<std::endl<<angle<<std::endl;
    Vector2f sliderPos;
    if(doInvKin(sliderPos, angle)!=0)
    {
        std::stringstream err;
        err << "Could not find inv kinematic solution: " << angle(0) << ", " << angle(1);
        RCS::Logger::log("gov.nasa.robonet.WristMechanism", log4cpp::Priority::DEBUG, err.str());
        throw std::runtime_error(err.str());
    }

    return sliderPos;
}


/**
  * @brief records the current encoder value as tared value
  * @param encoder the current encoder value
  * @note encoderTarePos encoder reading when the encoder was tared
  **/
void WristMechanism::tareWristEncoders(Vector2f encoder)
{
    this->encoderTarePos = encoder;
    return;
}

/**
  * @brief the routine used to calibrate the wrist encoders
  * @param encoder current relative position encoder reading for the wrist actuators
  * @param sliderPos output from the inverse kinematics (getSliderFromAngle).  This signal is updated when the wrist is calibrated and gives the position of the sliders (in encoder counts) based on halls
  * @param angle the angle input into the inverse kinematics to get the slider pos (or known value)
  * @note encoderTared relative position of the encoder from where it was last tared
  * @note taredWristEncoderPos current absolute position of the sliders in counts
  * @note calSliderPos current absolute position of the sliders in millimeters, where zero is all the way in
  * @note angle current wrist angle in rad
  **/
void WristMechanism::calWrist(Vector2f encoder, Vector2f sliderPos, Vector2f angle)
{
    tareWristEncoders(encoder);
    this->calSliderPos(0) = (sliderPos(0) - sliderOffset(0))/sliderGain(0);
    this->calSliderPos(1) = (sliderPos(1) - sliderOffset(1))/sliderGain(1);
    //std::cout<<"calSliderPos: "<<std::endl<<calSliderPos<<std::endl;
    isCalibrated=true;
    lastAng = angle;

    return;
}

/**
  * @brief computes the rotation matrix for a given axis and angle
  * @param axis the axis about which the rotation is performed
  * @param theta the angle about which the rotation is performed
  * @throws runtime_error if the normal of axis is 0
  * @returns a 3x3 rotation matrix
  **/
Matrix3f WristMechanism::AA2Rot(Vector3f axis, float theta)
{
    Matrix3f rot = Matrix3f::Zero();
    //convert axis to unit vector
    float norm = axis.norm();
    if(norm==0)
    {
        std::stringstream err;
        err << "in AA2Rot(axis, theta) could not find find unit vector for axis (axis.norm() = 0)";
        RCS::Logger::log("gov.nasa.robonet.WristMechanism", log4cpp::Priority::ERROR, err.str());
        throw std::runtime_error(err.str());
    }

    Vector3f unitAxis = 1/norm*axis;

    float c = cos(theta);
    float s = sin(theta);
    float v = 1-c;
    float kx = unitAxis(0);
    float ky = unitAxis(1);
    float kz = unitAxis(2);

    rot(0,0) = (kx*kx*v) + c;
    rot(1,0) = (kx*ky*v) + (kz*s);
    rot(2,0) = (kx*kz*v) - (ky*s);

    rot(0,1) = (kx*ky*v) - (kz*s);
    rot(1,1) = (ky*ky*v) + c;
    rot(2,1) = (ky*kz*v) + (kx*s);

    rot(0,2) = (kx*kz*v) + (ky*s);
    rot(1,2) = (ky*kz*v) - (kx*s);
    rot(2,2) = (kz*kz*v) + c;


    return rot;
}


/**
  * @brief computes the rotation matrix for a given axis and RotationType
  * @param axis the axis about which the rotation is performed
  * @param RotationType a rotation type (ONE, SIN, COS)
  * @throws runtime_error if the normal of axis is 0
  * @throws runtime_error if RotationType is not defined
  * @returns a 3x3 rotation matrix
  **/
Matrix3f WristMechanism::AA2Rot(Vector3f axis, RotationType index)
{
    Matrix3f rot = Matrix3f::Zero();
    //convert axis to unit vector
    float norm = axis.norm();
    if(norm==0)
    {
        std::stringstream err;
        err << "in AA2Rot(axis, index) could not find find unit vector for axis (axis.norm() = 0)";
        RCS::Logger::log("gov.nasa.robonet.WristMechanism", log4cpp::Priority::ERROR, err.str());
        throw std::runtime_error(err.str());
    }

    Vector3f unitAxis = 1/norm*axis;

    float c, s, v;
    switch(index)
    {
    case ONE:
        c = 0;
        s = 0;
        v = 1;
        break;
    case SIN:
        c = 0;
        s = 1;
        v = 0;
        break;
    case COS:
        c = 1;
        s = 0;
        v =-1;
        break;
    default:
        std::stringstream err;
        err << "AA2Rot Rotation type not defined ";
        RCS::Logger::log("gov.nasa.robonet.WristMechanism", log4cpp::Priority::ERROR, err.str());
        throw std::invalid_argument(err.str());
    }

    float kx = unitAxis(0);
    float ky = unitAxis(1);
    float kz = unitAxis(2);

    rot(0,0) = (kx*kx*v) + c;
    rot(1,0) = (kx*ky*v) + (kz*s);
    rot(2,0) = (kx*kz*v) - (ky*s);

    rot(0,1) = (kx*ky*v) - (kz*s);
    rot(1,1) = (ky*ky*v) + c;
    rot(2,1) = (ky*kz*v) + (kx*s);

    rot(0,2) = (kx*kz*v) + (ky*s);
    rot(1,2) = (ky*kz*v) - (kx*s);
    rot(2,2) = (kz*kz*v) + c;

    return rot;
}

/**
  * @brief performs Euler predition and Newton correction by solving a local linearization of the problem
  * @param ang the output joint angles
  * @param pos the input slider values
  * @returns the correction to the joint angles
  **/
Vector2f WristMechanism::NewtonsMethod(Vector2f ang, Vector2f pos)
{
    WristMechanism::Partials Pabpo = findPartialDerivatives(alpha,beta,u,pos(0), ang);
    WristMechanism::Partials Pdcpo = findPartialDerivatives(delta,gamma,v,pos(1), ang);

    Matrix2f Fy;
    Fy <<   Pabpo.ftheta, Pabpo.fphi,
            Pdcpo.ftheta, Pdcpo.fphi;

    Vector2f Fxy;
    Fxy <<  Pabpo.f - lengthSq,
            Pdcpo.f - lengthSq;


//    std::cout<<"Fxy" <<std::endl<<Fxy<<std::endl;

//    MatrixXf A = MatrixXf::Random(20,20);
//    VectorXf b = VectorXf::Random(20);
//    VectorXf x;
//    JacobiSVD<MatrixXf> svdOfA(A);
//    x=svdOfA.solve(b);


//    JacobiSVD<Matrix2f> svd(Fy);
//    Vector2f dy = svd.solve(Fxy);
    Vector2f dy = Fy.inverse() * Fxy;

    return dy;
}

WristMechanism::Partials WristMechanism::findPartialDerivatives(Vector3f a, Vector3f b, Vector3f w, float q, Vector2f ang)
{
    WristMechanism::Partials wrist;

    Matrix3f Rmtheta = AA2Rot(m, ang(0));
    Matrix3f Rnphi = AA2Rot(n, ang(1));

    Vector3f r1 = Rnphi*b;
    Vector3f r2 = Rmtheta*(p+r1);
    Vector3f r = r2-a-q*w;
    Vector3f r0 = r2-a;
    wrist.f0 = r0.dot(r0);
    wrist.f = r.dot(r);
    wrist.fq = -2*r.dot(w);
    wrist.ftheta = 2*r.dot(m.cross(r2));
    wrist.fphi = 2*r.dot(Rmtheta*(n.cross(r1)));

    return wrist;
}

