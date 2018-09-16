#include <gtest/gtest.h>
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include "nasa_robodyn_mechanisms_core/FingerController.h"
#include <ros/package.h>

class FingerControllerTest : public ::testing::Test {
    protected:
        virtual void SetUp()
        {
        }

        virtual void TearDown()
        {
        }

        FingerController<3> finger;
        FingerKinematics<3> fingerKin;
};

TEST_F(FingerControllerTest, Controller)
{
    double DEG2RAD = 3.14159/180.;
    // set parameters
    FingerController<3>::ReferenceMatrixType refMat;
    refMat(0,0) = -3.78381327904;
    refMat(0,1) = -3.78381327904;
    refMat(0,2) = 3.78381327904;
    refMat(0,3) = 3.78381327904;
    refMat(1,0) = -4.94806351875;
    refMat(1,1) = 6.7308216983;
    refMat(1,2) = -4.94806351875;
    refMat(1,3) = 6.7308216983;
    refMat(2,0) = 0;
    refMat(2,1) = 0;
    refMat(2,2) = -4.94806351875;
    refMat(2,3) = 4.94806351875;
    finger.setReferenceMatrix(refMat);

    double mmPerCount = 0.0174427;
    finger.setMillimetersPerCount(mmPerCount);
    double hallScale = 0.000076295;
    FingerController<3>::JointVectorType hallCoeffs0, hallCoeffs1, hallCoeffs2, hallCoeffs3;
    hallCoeffs0[0] = 1.3025;
    hallCoeffs0[1] = 3.4872;
    hallCoeffs0[2] = 12.4459;
    hallCoeffs1[0] = -0.6444;
    hallCoeffs1[1] = -1.7247;
    hallCoeffs1[2] = -10.2433;
    hallCoeffs2[0] = 0.0528;
    hallCoeffs2[1] = 0.3821;
    hallCoeffs2[2] = 3.3336;
    hallCoeffs3[0] = -0.0039;
    hallCoeffs3[1] = -0.0463;
    hallCoeffs3[2] = -0.4100;
    finger.setHallAngleParameters(hallScale, hallCoeffs0, hallCoeffs1, hallCoeffs2, hallCoeffs3);
    
    FingerController<3>::SliderVectorType aGains, bGains, tensionOffsets, calstrain;
    aGains[0] =  0.01;
    aGains[1] =  -0.01;
    aGains[2] =  0.01;
    aGains[3] =  0.01;
    bGains[0] =  -0.01;
    bGains[1] =  0.01;
    bGains[2] =  -0.01;
    bGains[3] =  -0.01;
    tensionOffsets[0] =  0.969;
    tensionOffsets[1] =  -1.132;
    tensionOffsets[2] =  3.743;
    tensionOffsets[3] =  -4.896;
    calstrain[0] =  1.;
    calstrain[1] =  1.;
    calstrain[2] =  1.;
    calstrain[3] =  1.;
    finger.setTensionParameters(aGains, bGains, tensionOffsets, calstrain);
    
    FingerController<3>::SliderVectorType encoders, sliders, sliderPos, tensions;
    encoders = FingerController<3>::SliderVectorType::Zero();
    sliders[0] = -112;
    sliders[1] = 65;
    sliders[2] = -213;
    sliders[3] = 10;
    sliderPos[0] = -31.73*mmPerCount;
    sliderPos[1] = 111.5*mmPerCount;
    sliderPos[2] = -74.37*mmPerCount;
    sliderPos[3] = 38.32*mmPerCount;
    tensions[0] = 0.09;
    tensions[1] = 2.837;
    tensions[2] = 7.84;
    tensions[3] = -1.265;
    finger.setTubeTareParameters(sliders, sliderPos, tensions);
    
    MultiLoopController mlc;
    mlc.setLoopRate(350);
    mlc.setPositionLoopParameters(0, 0, 0);
    mlc.setVelocityLoopParameters(0, 0, 0, 0, 0);
    mlc.setCurrentLoopParameters(-.25, .25, 200);
    mlc.setHardwareParameters(0, 0, .00742, 15.1, .007419811, 0, 1, 50000, .000002, .0000002);
    finger.setMultiLoopController(mlc, 0);
    finger.setMultiLoopController(mlc, 1);
    finger.setMultiLoopController(mlc, 2);
    finger.setMultiLoopController(mlc, 3);
    finger.setBusVoltage(48);

    encoders[0] = -42;
    encoders[1] = -42;
    encoders[2] = -37;
    encoders[3] = -36;
    finger.getSlidersFromEncoders(encoders, sliders);
    EXPECT_NEAR(38.27*mmPerCount, sliders[0], 0.1);
    EXPECT_NEAR(4.514*mmPerCount, sliders[1], 0.1);
    EXPECT_NEAR(101.6*mmPerCount, sliders[2], 0.1);
    EXPECT_NEAR(-7.68*mmPerCount, sliders[3], 0.1);

    FingerController<3>::JointVectorType joints;
    finger.getJointsFromSliders(sliders, joints);
    EXPECT_NEAR(3.378*DEG2RAD, joints[0], 0.01);
    EXPECT_NEAR(-1.054*DEG2RAD, joints[1], 0.01);
    EXPECT_NEAR(-9.795*DEG2RAD, joints[2], 0.01);

    FingerController<3>::JointVectorType halls;
    halls[0] = 33220;
    halls[1] = 47300;
    halls[2] = 44530;
    finger.getHallAngles(halls, joints);
    EXPECT_NEAR(-0.0538, joints[0], 0.017);
    EXPECT_NEAR(0.0602, joints[1], 0.017);
    EXPECT_NEAR(0.0591, joints[2], 0.017);

    joints[0] = 0*DEG2RAD;
    joints[1] = 5*DEG2RAD;
    joints[2] = 2*DEG2RAD;
    finger.getSlidersFromJoints(joints, sliders);
    EXPECT_NEAR(-0.017*25.4, sliders[0], 0.001);
    EXPECT_NEAR(0.02313*25.4, sliders[1], 0.001);
    EXPECT_NEAR(-0.0238*25.4, sliders[2], 0.001);
    EXPECT_NEAR(0.02992*25.4, sliders[3], 0.001);
    
    FingerController<3>::SliderVectorType tensionA, tensionB, tensionOut;
    tensionA[0] = 32670;
    tensionA[1] = 32640;
    tensionA[2] = 33030;
    tensionA[3] = 32850;
    tensionB[0] = 32690;
    tensionB[1] = 33150;
    tensionB[2] = 32590;
    tensionB[3] = 32150;
    finger.getCalibratedTensions(tensionA, tensionB, 0.0, tensionOut);
    EXPECT_NEAR(0.719, tensionOut[0], 0.1);
    EXPECT_NEAR(1.061, tensionOut[1], 0.1);
    EXPECT_NEAR(0.243, tensionOut[2], 0.1);
    EXPECT_NEAR(3.289, tensionOut[3], 0.1);
}

TEST_F(FingerControllerTest, RangeSpace)
{
    // set parameters
    FingerController<3>::ReferenceMatrixType refMat;
    refMat(0,0) = -3.78381327904;
    refMat(0,1) = -3.78381327904;
    refMat(0,2) = 3.78381327904;
    refMat(0,3) = 3.78381327904;
    refMat(1,0) = -4.94806351875;
    refMat(1,1) = 6.7308216983;
    refMat(1,2) = -4.94806351875;
    refMat(1,3) = 6.7308216983;
    refMat(2,0) = 0;
    refMat(2,1) = 0;
    refMat(2,2) = -4.94806351875;
    refMat(2,3) = 4.94806351875;
    fingerKin.setReferenceMatrix(refMat);

    FingerController<3>::SliderVectorType sliders;
    sliders[0] = -0.01837;
    sliders[1] = -0.02344;
    sliders[2] = -0.0483;
    sliders[3] = -0.006295;
    fingerKin.projectToRangeSpace(sliders, sliders);
    EXPECT_NEAR(0.0112, sliders[0], 0.015);
    EXPECT_NEAR(-0.01411, sliders[1], 0.015);
    EXPECT_NEAR(-0.03641, sliders[2], 0.015);
    EXPECT_NEAR(0.002753, sliders[3], 0.015);

    sliders[0] = -0.3396;
    sliders[1] = 0.4174;
    sliders[2] = -0.7117;
    sliders[3] = 0.7709;
    fingerKin.projectToRangeSpace(sliders, sliders);
    EXPECT_NEAR(-0.3103, sliders[0], 0.015);
    EXPECT_NEAR(0.4267, sliders[1], 0.015);
    EXPECT_NEAR(-0.6999, sliders[2], 0.015);
    EXPECT_NEAR(0.7799, sliders[3], 0.015);
}

TEST_F(FingerControllerTest, CheckHWNumbers)
{
    // set parameters
    FingerController<3>::ReferenceMatrixType refMat;
    refMat(0,0) = 3.78381327904;
    refMat(0,1) = 3.78381327904;
    refMat(0,2) = -3.78381327904;
    refMat(0,3) = -3.78381327904;
    refMat(1,0) = 4.94806351875;
    refMat(1,1) = -6.7308216983;
    refMat(1,2) = 4.94806351875;
    refMat(1,3) = -6.7308216983;
    refMat(2,0) = 0;
    refMat(2,1) = 0;
    refMat(2,2) = 4.94806351875;
    refMat(2,3) = -4.94806351875;
    finger.setReferenceMatrix(refMat);

    double mmPerCount = 0.0174427;
    finger.setMillimetersPerCount(mmPerCount);
    FingerController<3>::SliderVectorType encoders, sliders, sliderPos, tensions;
    encoders = FingerController<3>::SliderVectorType::Zero();
    sliders[0] = -78;
    sliders[1] = -4;
    sliders[2] = -52;
    sliders[3] = -6;
    sliderPos[0] = 5.1545;
    sliderPos[1] = -10.6512;
    sliderPos[2] = 8.861;
    sliderPos[3] = -8.1897;
    finger.setTubeTareParameters(sliders, sliderPos, tensions);

    encoders[0] = -78;
    encoders[1] = -4;
    encoders[2] = -52;
    encoders[3] = -6;
    finger.getSlidersFromEncoders(encoders, sliders);
    EXPECT_NEAR(5.1545, sliders[0], 0.1);
    EXPECT_NEAR(-10.6512, sliders[1], 0.1);
    EXPECT_NEAR(8.861, sliders[2], 0.1);
    EXPECT_NEAR(-8.1897, sliders[3], 0.1);

    FingerController<3>::JointVectorType joints;
    finger.getJointsFromSliders(sliders, joints);
    EXPECT_NEAR(-0.411, joints[0], 0.02);
    EXPECT_NEAR(1.3423, joints[1], 0.02);
    EXPECT_NEAR(0.141, joints[2], 0.02);

    joints[0] = -0.411;
    joints[1] = 1.3423;
    joints[2] = 0.141;
    finger.getSlidersFromJoints(joints, sliders);
    EXPECT_NEAR(5.1545, sliders[0], 0.1);
    EXPECT_NEAR(-10.6512, sliders[1], 0.1);
    EXPECT_NEAR(8.861, sliders[2], 0.1);
    EXPECT_NEAR(-8.1897, sliders[3], 0.1);

    joints[0] = -0.411;
    joints[1] = .5;
    joints[2] = 0.141;
    finger.getSlidersFromJoints(joints, sliders);
    EXPECT_GE(5.1545, sliders[0]);
    EXPECT_LE(-10.6512, sliders[1]);
    EXPECT_GE(8.861, sliders[2]);
    EXPECT_LE(-8.1897, sliders[3]);
}

int main(int argc, char **argv){
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
