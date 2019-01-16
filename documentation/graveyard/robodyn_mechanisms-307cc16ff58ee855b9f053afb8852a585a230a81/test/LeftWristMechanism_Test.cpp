#include <iostream>
#include <gtest/gtest.h>
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include "nasa_robodyn_mechanisms_core/WristMechanism.h"
#include <ros/package.h>
#include <stdexcept>

using namespace Eigen;

class LeftWristMechanismTest : public ::testing::Test {
protected:
    virtual void SetUp()
    {
        leftWrist = boost::make_shared<WristMechanism>();

//        //! values for cal = 0
//        A  << -26.40, 30, 191;
//        A0 << -26.93, 30, 174.85;
//        B  << -27.47,31.77,231;
//        C  << -27.47,-26.43,246;
//        D  << -25.75,-30, 210;
//        D0 << -25.93, -30, 204.72;
//        sliderZeroKnown = Vector2f(15.41, 0.667);
//        sliderCalKnown = Vector2f(0, 0);


//        //! values for cal = 30
//        A  << -26.40, 30, 191;
//        A0 << -27.91, 30, 144.86;
//        B  << -27.47,31.77,231;
//        C  << -27.47,-26.43,246;
//        D  << -25.92, -30, 205;
//        D0 << -26.93, -30, 174.73;
//        sliderZeroKnown = Vector2f(45.41, 30.67);
//        sliderCalKnown = Vector2f(30, 30);

//        //! values for cal = 46
//        //! @note 46 is the only value that makes fwd and inv kinematics agree in the range test
//        A  << -26.40, 30, 191;
//        A0 << -28.43, 30, 128.87;
//        B  << -27.47,31.77,231;
//        C  << -27.47,-26.43,246;
//        D  << -25.92, -30, 205;
//        D0 << -27.46, -30, 158.74;
//        sliderZeroKnown = Vector2f(61.4, 46.7);
//        sliderCalKnown = Vector2f(46, 46);

        //! values for cal = 46 -- moved zero point to match model
        //! @note 46 is the only value that makes fwd and inv kinematics agree in the range test
        A  << -26.35, 30, 192.4;
        A0 << -28.39, 30, 130.27;
        B  << -27.47,31.77,231;
        C  << -27.47,-26.43,246;
        D  << -25.75, -30, 210.2;
        D0 << -27.29, -30, 163.94;
        sliderZeroKnown = Vector2f(60, 41.5);
        sliderCalKnown = Vector2f(46, 46);

        Pitch << 0,0,222;
        Yaw   << 0,0,228;
        PalmCenter << 0,0,281;
        M  << 0,1,0;
        N  << 1,0,0;
        maxTravel = 100;
        linkLength = 40.8;

        leftWrist->loadDesignParams(A,A0,B,C,D,D0,Pitch,Yaw,M,N,linkLength);

        upperPitch = 1.2;
        lowerPitch = -1.2;
        upperYaw = 0.26;
        lowerYaw = -.78;

        leftWrist->setLimits(upperPitch,lowerPitch,upperYaw,lowerYaw);

        SliderGain(0) = .003906;
        SliderGain(1) = .003906;
        leftWrist->setSliderOffsetGain(Vector2f::Zero(), SliderGain);
        leftWrist->setAngleOffsetGain(Vector2f::Zero(), Vector2f(1,1));
        leftWrist->setLimitOffsetGain(1.2,.586, -1.2, 0, 0.26, 0, -1.9, -1.42 );

        //angleCalKnown = Vector2f(-0.27, -0.252);
        angleCalKnown = Vector2f(-0.1473, -0.3136);
        encoder = Vector2f::Zero();

        angZeroKnown = Vector2f::Zero();

    }

    virtual void TearDown()
    {
    }

    boost::shared_ptr<WristMechanism> leftWrist;

    Vector3f A;
    Vector3f A0;
    Vector3f B;
    Vector3f C;
    Vector3f D;
    Vector3f D0;
    Vector3f Pitch;
    Vector3f Yaw;
    Vector3f PalmCenter;
    Vector3f M;
    Vector3f N;
    float maxTravel;
    float linkLength;

    float upperPitch;
    float lowerPitch;
    float upperYaw;
    float lowerYaw;
    float yawOffset;

    Vector2f SliderGain;
    Vector2f sliderCalKnown;
    Vector2f angleCalKnown;
    Vector2f encoder;
    Vector2f sliderZeroKnown;
    Vector2f angZeroKnown;

};

TEST_F(LeftWristMechanismTest, setBadParamsTest)
{

    leftWrist->outputDesignParams();
    //A and A0 cannot be the same
    EXPECT_THROW(leftWrist->loadDesignParams(A,A,B,C,D,D0,Pitch,Yaw,M,N,linkLength), std::runtime_error);

    //D and D0 cannot be the same
    EXPECT_THROW(leftWrist->loadDesignParams(A,A0,B,C,D,D,Pitch,Yaw,M,N,linkLength), std::runtime_error);

    Vector3f Zero = Vector3f::Zero();;

    //M cannot be a zero vector
    EXPECT_THROW(leftWrist->loadDesignParams(A,A0,B,C,D,D0,Pitch,Yaw,Zero,N,linkLength), std::runtime_error);

    //N cannot be a zero vector
    EXPECT_THROW(leftWrist->loadDesignParams(A,A0,B,C,D,D0,Pitch,Yaw,M,Zero,linkLength), std::runtime_error);

}

TEST_F(LeftWristMechanismTest, applyLimitsTest)
{

    // set value within limits
    float pitch = 0;
    float yaw = 0;
    leftWrist->applyLimits(pitch, yaw);

    EXPECT_FLOAT_EQ(0, pitch);
    EXPECT_FLOAT_EQ(0, yaw);

    // set pitch outside of limits
    pitch = upperPitch+.1;
    yaw = 0;
    leftWrist->applyLimits(pitch,yaw);

    EXPECT_FLOAT_EQ(upperPitch, pitch);
    EXPECT_FLOAT_EQ(0, yaw);

    pitch = lowerPitch - .1;
    leftWrist->applyLimits(pitch,yaw);

    EXPECT_FLOAT_EQ(lowerPitch, pitch);
    EXPECT_FLOAT_EQ(0, yaw);

    // set yaw outside of limits
    pitch = 0;
    yaw = upperYaw+.1;
    leftWrist->applyLimits(pitch,yaw);

    EXPECT_FLOAT_EQ(0, pitch);
    EXPECT_FLOAT_EQ(upperYaw, yaw);

    yaw = lowerYaw-.1;
    leftWrist->applyLimits(pitch,yaw);

    EXPECT_FLOAT_EQ(0, pitch);
    EXPECT_FLOAT_EQ(lowerYaw, yaw);

}


TEST_F(LeftWristMechanismTest, getSliderFromAngleTest)
{
    //checks inverse kinematics

    leftWrist->calWrist(encoder,  sliderCalKnown, angleCalKnown);

    //check cal position
    std::cout<<"angleCalKnown: ("<<angleCalKnown(0)<<","<<angleCalKnown(1)<<")"<<std::endl;
    Vector2f sliderCalPos = leftWrist->getSliderFromAngle(angleCalKnown);
    std::cout<<"slider pos:"<<std::endl<<sliderCalPos<<std::endl;
    EXPECT_NEAR(sliderCalKnown(0), sliderCalPos(0),  1);
    EXPECT_NEAR(sliderCalKnown(1), sliderCalPos(1),  1);

    //check zero position
    Vector2f sliderZeroPos = leftWrist->getSliderFromAngle(Vector2f::Zero());

    std::cout<<"slider pos:"<<std::endl<<sliderZeroPos<<std::endl;
    EXPECT_NEAR(sliderZeroKnown(0), sliderZeroPos(0), 1);
    EXPECT_NEAR(sliderZeroKnown(1), sliderZeroPos(1), 1);

    Vector2f ang = Vector2f::Zero();
    //change pitch -- increase of pitch angle should increase both sliders
    ang(0) = 0.0872664626; //5 degrees
    Vector2f slider = leftWrist->getSliderFromAngle(ang);
    std::cout<<"slider pos:"<<std::endl<<slider<<std::endl;
    EXPECT_LT(sliderZeroKnown(0), slider(0));
    EXPECT_LT(sliderZeroKnown(1), slider(1));


    ang = Vector2f::Zero();
    //change pitch -- decrease of pitch angle should decrease both sliders
    ang(0) = -0.0872664626; //-5 degrees
    slider = leftWrist->getSliderFromAngle(ang);
    std::cout<<"slider pos:"<<std::endl<<slider<<std::endl;
    EXPECT_GT(sliderZeroKnown(0), slider(0));
    EXPECT_GT(sliderZeroKnown(1), slider(1));

    ang = Vector2f::Zero();
    //change yaw -- increase of yaw should increase thumbside and decrease littleside
    ang(1) = 0.0872664626; //5 degreess
    slider = leftWrist->getSliderFromAngle(ang);
    std::cout<<"slider pos:"<<std::endl<<slider<<std::endl;

    EXPECT_LT(sliderZeroKnown(0), slider(0));
    EXPECT_GT(sliderZeroKnown(1), slider(1));

    ang = Vector2f::Zero();
    //change yaw -- decrease of yaw should decrease thumside and increase littleside
    ang(1) = -0.0872664626; // -5 degrees
    slider = leftWrist->getSliderFromAngle(ang);
    std::cout<<"slider pos:"<<std::endl<<slider<<std::endl;
    EXPECT_GT(sliderZeroKnown(0), slider(0));
    EXPECT_LT(sliderZeroKnown(1), slider(1));


}

/// @TODO Test fails on 32-bit machines
TEST_F(LeftWristMechanismTest, sliderWristConversionTest)
{

    //initialize
    Vector2f ang = Vector2f::Zero();
    Vector2f slider = leftWrist->getSliderFromAngle(ang);
    Vector2f encoder = Vector2f::Zero();
    leftWrist->calWrist(encoder, slider, ang);

    //test forward and back give same result
    Vector2f slider2 = leftWrist->getSliderFromWristEncoder(encoder);
    EXPECT_FLOAT_EQ(slider(0), slider2(0));
    EXPECT_FLOAT_EQ(slider(1), slider2(1));

    Vector2f encoder2 = leftWrist->getWristEncoderFromSlider(slider);
    EXPECT_FLOAT_EQ(encoder(0), encoder2(0));
    EXPECT_FLOAT_EQ(encoder(1), encoder2(1));

    //test increased encoder is increased sliderpos
    Vector2f constant = Vector2f::Constant(10);
    Vector2f encoderInc = encoder + constant;
    Vector2f sliderInc = leftWrist->getSliderFromWristEncoder(encoderInc);
    EXPECT_GT(sliderInc(0), slider(0));
    EXPECT_GT(sliderInc(1), slider(1));

    //test decreased encoder decreases sliderpos
    Vector2f encoderDec = encoder - constant;
    Vector2f sliderDec = leftWrist->getSliderFromWristEncoder(encoderDec);
    EXPECT_LT(sliderDec(0), slider(0));
    EXPECT_LT(sliderDec(1), slider(1));

    //test increasing sliderpos increases encoder
    sliderInc = slider + constant;
    encoderInc = leftWrist->getWristEncoderFromSlider(sliderInc);
    EXPECT_GT(encoderInc(0), encoder(0));
    EXPECT_GT(encoderInc(1), encoder(1));

    //test decreasing sliderpos decreases encoder
    sliderDec = slider - constant;
    encoderDec = leftWrist->getWristEncoderFromSlider(sliderDec);
    EXPECT_LT(encoderDec(0), encoder(0));
    EXPECT_LT(encoderDec(1), encoder(1));

}

TEST_F(LeftWristMechanismTest, NewtonsMethodTest)
{
    Vector2f angKnown = Vector2f(0.0070703288, 0.00459021593);
    Vector2f ang = Vector2f::Zero();

    leftWrist->eps = 0.0001;
    Vector2f slider = leftWrist->getSliderFromAngle(angKnown);
    Vector2f correction;
    for(int i = 0; i<10; ++i)
    {
        correction = leftWrist->NewtonsMethod(ang, slider);
        ang = ang - correction;
    }

    EXPECT_NEAR(angKnown(0), ang(0), .01);
    EXPECT_NEAR(angKnown(1), ang(1), .01);


    ang = Vector2f::Zero();
    //change pitch (increase)
    ang(0) = 0.0872664626; //5 degrees
    for(int i = 0; i<1000; ++i)
    {
        correction = leftWrist->NewtonsMethod(ang, slider);
        ang = ang - correction;
    }

    EXPECT_NEAR(angKnown(0), ang(0), .01);
    EXPECT_NEAR(angKnown(1), ang(1), .01);

    //change pitch(decrease)
    ang(0) = -0.0872664626; //-5 degrees
    for(int i = 0; i<1000; ++i)
    {
        correction = leftWrist->NewtonsMethod(ang, slider);
        ang = ang - correction;
    }
    EXPECT_NEAR(angKnown(0), ang(0), .01);
    EXPECT_NEAR(angKnown(1), ang(1), .01);

    ang = Vector2f::Zero();

    //change yaw (increase)
    ang(1) = 0.0872664626; //5 degrees
    for(int i = 0; i<1000; ++i)
    {
        correction = leftWrist->NewtonsMethod(ang, slider);
        ang = ang - correction;
    }
    EXPECT_NEAR(angKnown(0), ang(0), .01);
    EXPECT_NEAR(angKnown(1), ang(1), .01);

    //change yaw (decrease)
    ang(1) = -0.0872664626; //-5 degrees
    for(int i = 0; i<1000; ++i)
    {
        correction = leftWrist->NewtonsMethod(ang, slider);
        ang = ang - correction;
    }
    EXPECT_NEAR(angKnown(0), ang(0), .01);
    EXPECT_NEAR(angKnown(1), ang(1), .01);

}

TEST_F(LeftWristMechanismTest, getAngleFromSliderTest)
{
    //checks fwd kinematics

    EXPECT_NO_THROW(leftWrist->calWrist(encoder,  sliderCalKnown, angleCalKnown));
    leftWrist->maxIt = 150;
    leftWrist->eps = .0001;

    Vector2f angOut;
    //check calibration point
    EXPECT_NO_THROW(angOut = leftWrist->getAngleFromSlider(sliderCalKnown));
    std::cout<<"angle:"<<std::endl<<angOut<<std::endl;
    EXPECT_NEAR(angleCalKnown(0), angOut(0), 0.005);
    EXPECT_NEAR(angleCalKnown(1), angOut(1), 0.005);

    //check zero pos
    EXPECT_NO_THROW(angOut = leftWrist->getAngleFromSlider(sliderZeroKnown));
    std::cout<<"angle:"<<std::endl<<angOut<<std::endl;
    EXPECT_NEAR(angZeroKnown(0), angOut(0), 0.005);
    EXPECT_NEAR(angZeroKnown(1), angOut(1), 0.005);

    //increasing both sliders should increase pitch
    Vector2f slider = sliderZeroKnown + Vector2f::Constant(10);
    EXPECT_NO_THROW(angOut = leftWrist->getAngleFromSlider(slider));
    std::cout<<"angle:"<<std::endl<<angOut<<std::endl;
    EXPECT_LT(angZeroKnown(0), angOut(0));

    //decreasing both sliders should decrease pitch
    slider = sliderZeroKnown - Vector2f::Constant(10);
    EXPECT_NO_THROW(angOut = leftWrist->getAngleFromSlider(slider));
    std::cout<<"angle:"<<std::endl<<angOut<<std::endl;
    EXPECT_GT(angZeroKnown(0), angOut(0));

    //decreasing thumbside, and increasing littleside should decrease yaw
    slider = sliderZeroKnown + Vector2f(-5,5);
    EXPECT_NO_THROW(angOut = leftWrist->getAngleFromSlider(slider));
    std::cout<<"angle:"<<std::endl<<angOut<<std::endl;
    EXPECT_GT(angZeroKnown(1), angOut(1));

    //increasing thumbside, and decreasing littleside should increase yaw
    slider = sliderZeroKnown + Vector2f(5,-5);
    EXPECT_NO_THROW(angOut = leftWrist->getAngleFromSlider(slider));
    std::cout<<"angle:"<<std::endl<<angOut<<std::endl;
    EXPECT_LT(angZeroKnown(1), angOut(1));

}

TEST_F(LeftWristMechanismTest, FindDesignSpaceTest)
{
    Vector2f maxSlider1 = Vector2f::Zero();
    Vector2f minSlider1 = Vector2f::Zero();
    Vector2f maxSlider2 = Vector2f::Zero();
    Vector2f minSlider2 = Vector2f::Zero();
    Vector2f maxPitch = Vector2f::Zero();
    Vector2f minPitch = Vector2f::Zero();
    Vector2f maxYaw = Vector2f::Zero();
    Vector2f minYaw = Vector2f::Zero();
    Vector2f sliderOut  = Vector2f::Zero();
    Vector2f angOut     = Vector2f::Zero();
    for(double i = lowerPitch; i<upperPitch; i+=.01)
    {
        for(double j = lowerYaw; j< upperYaw; j+=.01)
        {
            try
            {
                float pitch = i;
                float yaw = j;
                leftWrist->applyLimits(pitch,yaw);
                sliderOut = leftWrist->getSliderFromAngle(Vector2f(pitch,yaw));
                if(abs(sliderOut(0)-sliderOut(1))>30 || sliderOut(0)<0 || sliderOut(0)>80 || sliderOut(1)<0 || sliderOut(1)>80)
                {
                    //std::cout<<"angle ("<<i<<","<<j<<")"<<" violates slider limits: ("<<sliderOut(0)<<","<<sliderOut(1)<<")"<<std::endl;
                    continue;
                }
                if(maxSlider1(0)<sliderOut(0))
                    maxSlider1 = sliderOut;
                if(minSlider1(0)>sliderOut(0))
                    minSlider1 = sliderOut;
                if(maxSlider2(1)<sliderOut(1))
                    maxSlider2 = sliderOut;
                if(minSlider2(1)>sliderOut(1))
                    minSlider2 = sliderOut;
            }
            catch(std::runtime_error)
            {
                std::cout<<"could not find inv kinematic solution ("<<i<<","<<j<<")"<<std::endl;
                std::cin.ignore();
            }

            try
            {
                angOut = leftWrist->getAngleFromSlider(sliderOut);
                float pitch = i;
                float yaw = j;
                leftWrist->applyLimits(pitch, yaw);
//                if(fabs(pitch-angOut(0)) > 0.005)
//                {
//                    std::cout<<"angle ("<<pitch<<","<<yaw<<")"<<" and ("<<angOut(0)<<","<<angOut(1)<<") are more than 0.005 apart: ("<<sliderOut(0)<<","<<sliderOut(1)<<")"<<std::endl;
////                    std::cin.ignore();
//                }
//                if(fabs(yaw-angOut(1))>0.005)
//                {
//                    std::cout<<"angle ("<<pitch<<","<<yaw<<")"<<" and ("<<angOut(0)<<","<<angOut(1)<<") are more than 0.005 apart: ("<<sliderOut(0)<<","<<sliderOut(1)<<")"<<std::endl;
////                    std::cin.ignore();
//                }
                EXPECT_NEAR(pitch, angOut(0), 0.005);
                EXPECT_NEAR(yaw, angOut(1), 0.005);

                if(maxPitch(0)<angOut(0))
                    maxPitch = angOut;
                if(minPitch(0)>angOut(0))
                    minPitch = angOut;
                if(maxYaw(1)<angOut(1))
                    maxYaw = angOut;
                if(minYaw(1)>angOut(1))
                    minYaw = angOut;
            }
            catch(std::runtime_error)
            {
                std::cout<<"Fwd Kinematics not valid for ("<<i<<","<<j<<"), slider pos: ("<<sliderOut(0)<<","<<sliderOut(1)<<")"<<std::endl;
                std::cin.ignore();
            }


        }
    }

    std::cout<<"Max Slider1: "<<maxSlider1<<std::endl;
    std::cout<<"Min Slider1: "<<minSlider1<<std::endl;
    std::cout<<"Max Slider2: "<<maxSlider2<<std::endl;
    std::cout<<"Min Slider2: "<<minSlider2<<std::endl;
    std::cout<<"Max Pitch: "<<maxPitch<<std::endl;
    std::cout<<"Min Pitch: "<<minPitch<<std::endl;
    std::cout<<"Max Yaw: "<<maxYaw<<std::endl;
    std::cout<<"Min Yaw: "<<minYaw<<std::endl;
}
/*
TEST_F(LeftWristMechanismTest, FindRangeSpaceTest)
{

//    for(double i = 0; i < -100; i = i-.1)
//    {
//        std::cout<<"("<<i<<","<<i<<")"<<std::endl;
//        EXPECT_NO_THROW(leftWrist->getAngleFromSlider(Vector2f(i,i)));
//        leftWrist->getAngleFromSlider(Vector2f(i,i));
//    }

    Vector2f maxPitch = Vector2f::Zero();
    Vector2f minPitch = Vector2f::Zero();
    Vector2f maxYaw = Vector2f::Zero();
    Vector2f minYaw = Vector2f::Zero();
    Vector2f angOut = Vector2f::Zero();
    for(double i = -25; i < 95; i = i+.1)
    {
        //std::cout<<"("<<i+25<<","<<i<<")"<<std::endl;
        EXPECT_NO_THROW(angOut = leftWrist->getAngleFromSlider(Vector2f(i,i)));
        try
        {
            angOut = leftWrist->getAngleFromSlider(Vector2f(i+25,i));

            if(maxPitch(0)<angOut(0))
                maxPitch = angOut;
            if(minPitch(0)>angOut(0))
                minPitch = angOut;
            if(maxYaw(1)<angOut(1))
                maxYaw = angOut;
            if(minYaw(1)>angOut(1))
                minYaw = angOut;
        }
        catch(std::runtime_error)
        {
            continue;
        }


    }

    for(double i = -25; i < 95; i = i+.1)
    {
        for(double j = i-25; j<i+25; j = j+.1)
        {
            //std::cout<<"("<<i<<","<<j<<")"<<std::endl;
            //EXPECT_NO_THROW(leftWrist->getAngleFromSlider(Vector2f(i,j)));
            try
            {
                angOut = leftWrist->getAngleFromSlider(Vector2f(i,j));

                if(maxPitch(0)<angOut(0))
                    maxPitch = angOut;
                if(minPitch(0)>angOut(0))
                    minPitch = angOut;
                if(maxYaw(1)<angOut(1))
                    maxYaw = angOut;
                if(minYaw(1)>angOut(1))
                    minYaw = angOut;
            }
            catch(std::runtime_error)
            {
                continue;
            }
        }
    }

    for(double i = -25; i < 95; i = i+.1)
    {
        for(double j = i-25; j<i+25; j = j+.1)
        {
            //std::cout<<"("<<j<<","<<i<<")"<<std::endl;
            //EXPECT_NO_THROW(leftWrist->getAngleFromSlider(Vector2f(j,i)));
            try
            {
                angOut = leftWrist->getAngleFromSlider(Vector2f(j,i));
                if(maxPitch(0)<angOut(0))
                    maxPitch = angOut;
                if(minPitch(0)>angOut(0))
                    minPitch = angOut;
                if(maxYaw(1)<angOut(1))
                    maxYaw = angOut;
                if(minYaw(1)>angOut(1))
                    minYaw = angOut;
            }
            catch(std::runtime_error)
            {
                continue;
            }

        }
    }

    std::cout<<"Max Pitch: "<<maxPitch<<std::endl;
    std::cout<<"Min Pitch: "<<minPitch<<std::endl;
    std::cout<<"Max Yaw: "<<maxYaw<<std::endl;
    std::cout<<"Min Yaw: "<<minYaw<<std::endl;

}
*/
/*
TEST_F(LeftWristMechanismTest, RangeSpaceTest)
{

    Vector2f maxPitch = Vector2f::Zero();
    Vector2f minPitch = Vector2f::Zero();
    Vector2f maxYaw = Vector2f::Zero();
    Vector2f minYaw = Vector2f::Zero();
    Vector2f angOut = Vector2f::Zero();
    for(double i = 5; i < 75; i = i+.1)
    {
        EXPECT_NO_THROW(angOut = leftWrist->getAngleFromSlider(Vector2f(i,i)));

        if(maxPitch(0)<angOut(0))
            maxPitch = angOut;
        if(minPitch(0)>angOut(0))
            minPitch = angOut;
        if(maxYaw(1)<angOut(1))
            maxYaw = angOut;
        if(minYaw(1)>angOut(1))
            minYaw = angOut;
    }

    for(double i = 30; i < 70; i = i+.1)
    {
        for(double j = i-25; j<i+25; j = j+.1)
        {
            EXPECT_NO_THROW(angOut = leftWrist->getAngleFromSlider(Vector2f(i,j)));
            if(maxPitch(0)<angOut(0))
                maxPitch = angOut;
            if(minPitch(0)>angOut(0))
                minPitch = angOut;
            if(maxYaw(1)<angOut(1))
                maxYaw = angOut;
            if(minYaw(1)>angOut(1))
                minYaw = angOut;
        }
    }

    for(double i = 30; i <70; i = i+.1)
    {
        for(double j = i-25; j<i+25; j = j+.1)
        {
            EXPECT_NO_THROW(angOut = leftWrist->getAngleFromSlider(Vector2f(j,i)));
            if(maxPitch(0)<angOut(0))
                maxPitch = angOut;
            if(minPitch(0)>angOut(0))
                minPitch = angOut;
            if(maxYaw(1)<angOut(1))
                maxYaw = angOut;
            if(minYaw(1)>angOut(1))
                minYaw = angOut;
        }
    }

    std::cout<<"Max Pitch: "<<maxPitch<<std::endl;
    std::cout<<"Min Pitch: "<<minPitch<<std::endl;
    std::cout<<"Max Yaw: "<<maxYaw<<std::endl;
    std::cout<<"Min Yaw: "<<minYaw<<std::endl;
}

*/

TEST_F(LeftWristMechanismTest, calibration)
{

    leftWrist->calWrist(encoder,  sliderCalKnown, angleCalKnown);

    Vector2f sliderTest = leftWrist->getSliderFromAngle(angleCalKnown);
    EXPECT_NEAR(sliderCalKnown(0), sliderTest(0), 1);
    EXPECT_NEAR(sliderCalKnown(1), sliderTest(1), 1);

    Vector2f angOut = leftWrist->getAngleFromSlider(sliderCalKnown);
    EXPECT_NEAR(angleCalKnown(0), angOut(0), 0.005 );
    EXPECT_NEAR(angleCalKnown(1), angOut(1), 0.005 );

    Vector2f sliderOut = leftWrist->getSliderFromWristEncoder(encoder);
    EXPECT_NEAR(sliderCalKnown(0), sliderOut(0), 1);
    EXPECT_NEAR(sliderCalKnown(1), sliderOut(1), 1);

}


int main(int argc, char **argv){
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}


