#include <gtest/gtest.h>
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include "nasa_robodyn_mechanisms_core/GripperKinematics.h"

using namespace std;

//! @todo Make these tests worthwhile...

TEST(GripperKinematicsTest, Constructor)
{
    GripperKinematics gripperKinematics(0.688, 0.34, 0.477, 0.75, 0.733, 0.85,
                                        0.002, 0.438, 0.188, 1.625,
                                        0.0559, -0.0523598776, 0.01, 0.3,
                                        1e-6, false);
}

TEST(GripperKinematicsTest, GetEncoderAngle)
{
    GripperKinematics gripperKinematics(0.688, 0.34, 0.477, 0.75, 0.733, 0.85,
                                        0.002, 0.438, 0.188, 1.625,
                                        0.0559, -0.0523598776, 0.01, 0.3,
                                        1e-6, false);

    //! jaw angles range [-.05, 0.6]
    cout << gripperKinematics.getEncoderAngle(0.0) << endl;
}

TEST(GripperKinematicsTest, GetAngleOffSingular)
{
    GripperKinematics gripperKinematics(0.688, 0.34, 0.477, 0.75, 0.733, 0.85,
                                        0.002, 0.438, 0.188, 1.625,
                                        0.0559, -0.0523598776, 0.01, 0.3,
                                        1e-6, false);

    cout << gripperKinematics.getAngleOffSingular(0.0, 0.0) << endl;
}

TEST(GripperKinematicsTest, IsOverCenter)
{
    GripperKinematics gripperKinematics(0.688, 0.34, 0.477, 0.75, 0.733, 0.85,
                                        0.002, 0.438, 0.188, 1.625,
                                        0.0559, -0.0523598776, 0.01, 0.3,
                                        1e-6, false);

    cout << gripperKinematics.isOverCenter(0.0, 0.0) << endl;
 
    //! @todo this is a very hard test because the gripper model is inaccurate
    // double encoderAngle, jawAngle;

    // jawAngle = 0.05;
    // encoderAngle = gripperKinematics.getEncoderAngle(jawAngle);
    // cout << encoderAngle << " : "
    //      << jawAngle << " : "
    //      << gripperKinematics.getAngleOffSingular(encoderAngle, jawAngle) << " : "
    //      << gripperKinematics.isOverCenter(encoderAngle, jawAngle) << endl;

    // jawAngle = 0.05;
    // encoderAngle = gripperKinematics.getEncoderAngle(jawAngle);
    // encoderAngle -= 5;
    // cout << encoderAngle << " : "
    //      << jawAngle << " : "
    //      << gripperKinematics.getAngleOffSingular(encoderAngle, jawAngle) << " : "
    //      << gripperKinematics.isOverCenter(encoderAngle, jawAngle) << endl;

    // jawAngle = 0.05;
    // encoderAngle = gripperKinematics.getEncoderAngle(jawAngle);
    // encoderAngle += 5;
    // cout << encoderAngle << " : "
    //      << jawAngle << " : "
    //      << gripperKinematics.getAngleOffSingular(encoderAngle, jawAngle) << " : "
    //      << gripperKinematics.isOverCenter(encoderAngle, jawAngle) << endl;
}

TEST(GripperKinematicsTest, GetMotorCurrentLimit)
{
    GripperKinematics gripperKinematics(0.688, 0.34, 0.477, 0.75, 0.733, 0.85,
                                        0.002, 0.438, 0.188, 1.625,
                                        0.0559, -0.0523598776, 0.01, 0.3,
                                        1e-6, false);
                                        
    cout << gripperKinematics.getMotorCurrentLimit(0.0, 0.0, 0.0) << endl;
}

int main(int argc, char **argv){
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}