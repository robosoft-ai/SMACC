#include <gtest/gtest.h>
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include "nasa_r2_config_core/RobotInstanceFactory.h"
#include "nasa_r2_config_core/CoeffMapLoader.h"
#include "nasa_robodyn_mechanisms_core/JointCommandGripper.h"
#include <ros/package.h>

using namespace std;

std::map<std::string, float> testFloats;
CoeffMapPtr motorCoeffMap;
CoeffMapPtr brainstemCoeffMap;
RobotInstancePtr instance;

float getFloat(std::string item)
{
    return testFloats[item];
}

void setFloat(std::string item, float value)
{
    testFloats[item] = value;
}

float getMotorCoeff(std::string item)
{
    return motorCoeffMap->operator[](item);
}

std::string getCommandFile(std::string mechanism)
{
    return instance->configurationSafetyBasePath + instance->APIMAP_PATH + instance->mechanismCommandMap[mechanism];
}

class JointCommandGripperTest : public ::testing::Test {
    protected:
        virtual void SetUp()
        {
            packagePath      = ros::package::getPath("nasa_r2_config_core");
            configPath       = packagePath + "/test/config";
            configSafetyPath = packagePath + "/test/configSafety";
            fileName         = "TestRobotInstance.xml";
            instance         = RobotInstanceFactory::createRobotInstanceFromFile(configPath, configSafetyPath, fileName);

            motorCoeffMap = boost::make_shared<CoeffMap>();
            brainstemCoeffMap = boost::make_shared<CoeffMap>();
            CoeffMapLoader::loadElements(instance, motorCoeffMap, brainstemCoeffMap);

            testFloats.clear();

            io.getFloat         = getFloat;
            io.setFloat         = setFloat;
            io.getMotorCoeff    = getMotorCoeff;
            io.getCommandFile   = getCommandFile;

            ros::Time::init();
        }

        virtual void TearDown()
        {
        }

        string packagePath, configPath, configSafetyPath, fileName;
        JointCommandInterface::IoFunctions io;
};

TEST_F(JointCommandGripperTest, Constructor)
{
    ASSERT_NO_THROW(JointCommandGripper gripperJoint("/r2/right_leg/gripper/joint0", io));

    //! Missing IO function definitions
    io.getFloat = 0;
    ASSERT_THROW(JointCommandGripper gripperJoint("/r2/right_leg/gripper/joint0", io), std::invalid_argument);
}

TEST_F(JointCommandGripperTest, LoadCoeffs)
{
    JointCommandGripper gripperJoint("/r2/right_leg/gripper/joint0", io);

    EXPECT_NO_THROW(gripperJoint.loadCoeffs());

    nasa_r2_common_msgs::JointControlData controlMsg;
    controlMsg.coeffState.state = nasa_r2_common_msgs::JointControlCoeffState::LOADED;
    EXPECT_NO_THROW(gripperJoint.updateMeasuredState(controlMsg));

    nasa_r2_common_msgs::JointCommand          msg;
    controlMsg.controlMode.state = nasa_r2_common_msgs::JointControlMode::PARK;

    msg.header.stamp = ros::Time::now();

    msg.name.resize(1);
    msg.desiredPosition.resize(1);
    msg.desiredPositionVelocityLimit.resize(1);
    msg.feedForwardTorque.resize(1);
    msg.proportionalGain.resize(1);
    msg.derivativeGain.resize(1);
    msg.integralGain.resize(1);
    msg.positionLoopTorqueLimit.resize(1);
    msg.positionLoopWindupLimit.resize(1);
    msg.torqueLoopVelocityLimit.resize(1);

    msg.name[0]                         = "/r2/right_leg/gripper/joint0";
    msg.desiredPosition[0]              = 1;
    msg.desiredPositionVelocityLimit[0] = 2;
    msg.feedForwardTorque[0]            = 3;
    msg.proportionalGain[0]             = 4;
    msg.derivativeGain[0]               = 5;
    msg.integralGain[0]                 = 6;
    msg.positionLoopTorqueLimit[0]      = 7;
    msg.positionLoopWindupLimit[0]      = 8;
    msg.torqueLoopVelocityLimit[0]      = 9;

    EXPECT_NO_THROW(gripperJoint.setCommand(msg, controlMsg));

    EXPECT_NO_THROW(gripperJoint.getCommandedState());

    EXPECT_NO_THROW(gripperJoint.getCapability());
}

TEST_F(JointCommandGripperTest, getSimpleMeasuredState)
{
    JointCommandGripper gripperJoint("/r2/right_leg/gripper/joint0", io);

    setFloat("/r2/right_leg/gripper/joint0/EncPos", 1);
    setFloat("/r2/right_leg/gripper/joint0/EncVel", 2);
    setFloat("/r2/right_leg/gripper/joint0/APS1", 3);
    setFloat("/r2/right_leg/gripper/joint0/APS2", 5);
    setFloat("/r2/right_leg/gripper/joint0/JawLoad1", 7);
    setFloat("/r2/right_leg/gripper/joint0/JawLoad2", 8);

    nasa_r2_common_msgs::JointControlData controlMsg;
    controlMsg.coeffState.state = nasa_r2_common_msgs::JointControlCoeffState::LOADED;
    gripperJoint.updateMeasuredState(controlMsg);
    sensor_msgs::JointState msg = gripperJoint.getSimpleMeasuredState();

    EXPECT_EQ(3, msg.name.size());
    EXPECT_EQ(3, msg.position.size());
    EXPECT_EQ(3, msg.velocity.size());
    EXPECT_EQ(3, msg.effort.size());

    EXPECT_STREQ("/r2/right_leg/gripper/joint0",   msg.name[0].c_str());
    EXPECT_STREQ("/r2/right_leg/gripper/jawLeft",  msg.name[1].c_str());
    EXPECT_STREQ("/r2/right_leg/gripper/jawRight", msg.name[2].c_str());

    EXPECT_FLOAT_EQ(1, msg.position[0]);
    EXPECT_FLOAT_EQ(3, msg.position[1]);
    EXPECT_FLOAT_EQ(5, msg.position[2]);

    EXPECT_FLOAT_EQ(2, msg.velocity[0]);
    EXPECT_FLOAT_EQ(0, msg.velocity[1]);
    EXPECT_FLOAT_EQ(0, msg.velocity[2]);

    EXPECT_FLOAT_EQ(0, msg.effort[0]);
    EXPECT_FLOAT_EQ(7, msg.effort[1]);
    EXPECT_FLOAT_EQ(8, msg.effort[2]);
}

TEST_F(JointCommandGripperTest, getCommandedState)
{
    JointCommandGripper gripperJoint("/r2/right_leg/gripper/joint0", io);

    setFloat("/r2/right_leg/gripper/joint0/PosCom", 1);
    setFloat("/r2/right_leg/gripper/joint0/PosComVelocity", 2);
    setFloat("/r2/right_leg/gripper/joint0/TorqueCom", 3);
    setFloat("/r2/right_leg/gripper/joint0/JointStiffness", 4);
    setFloat("/r2/right_leg/gripper/joint0/JointDamping", 5);
    setFloat("/r2/right_leg/gripper/joint0/PLKi", 6);
    setFloat("/r2/right_leg/gripper/joint0/TorqueLim", 7);
    setFloat("/r2/right_leg/gripper/joint0/PLWindupLim", 8);
    setFloat("/r2/right_leg/gripper/joint0/TLVelLimit", 9);

    nasa_r2_common_msgs::JointCommand msg = gripperJoint.getCommandedState();

    EXPECT_EQ(0, msg.name.size());
    EXPECT_EQ(0, msg.desiredPosition.size());
    EXPECT_EQ(0, msg.desiredPositionVelocityLimit.size());
    EXPECT_EQ(0, msg.feedForwardTorque.size());
    EXPECT_EQ(0, msg.proportionalGain.size());
    EXPECT_EQ(0, msg.derivativeGain.size());
    EXPECT_EQ(0, msg.integralGain.size());
    EXPECT_EQ(0, msg.positionLoopTorqueLimit.size());
    EXPECT_EQ(0, msg.positionLoopWindupLimit.size());
    EXPECT_EQ(0, msg.torqueLoopVelocityLimit.size());

//    EXPECT_STREQ("/r2/right_leg/gripper/joint0", msg.name[0].c_str());
//    EXPECT_FLOAT_EQ(1, msg.desiredPosition[0]);
//    EXPECT_FLOAT_EQ(2, msg.desiredPositionVelocityLimit[0]);
//    EXPECT_FLOAT_EQ(3, msg.feedForwardTorque[0]);
//    EXPECT_FLOAT_EQ(4, msg.proportionalGain[0]);
//    EXPECT_FLOAT_EQ(5, msg.derivativeGain[0]);
//    EXPECT_FLOAT_EQ(6, msg.integralGain[0]);
//    EXPECT_FLOAT_EQ(7, msg.positionLoopTorqueLimit[0]);
//    EXPECT_FLOAT_EQ(8, msg.positionLoopWindupLimit[0]);
//    // EXPECT_FLOAT_EQ(9, msg.torqueLoopVelocityLimit[0]);
}

TEST_F(JointCommandGripperTest, setCommand)
{
    JointCommandGripper gripperJoint("/r2/right_leg/gripper/joint0", io);

    nasa_r2_common_msgs::JointCommand          msg;
    nasa_r2_common_msgs::JointControlData controlMsg;
    controlMsg.controlMode.state = nasa_r2_common_msgs::JointControlMode::PARK;

    msg.header.stamp = ros::Time::now();

    msg.name.resize(1);
    msg.desiredPosition.resize(1);
    msg.desiredPositionVelocityLimit.resize(1);
    msg.feedForwardTorque.resize(1);
    msg.proportionalGain.resize(1);
    msg.derivativeGain.resize(1);
    msg.integralGain.resize(1);
    msg.positionLoopTorqueLimit.resize(1);
    msg.positionLoopWindupLimit.resize(1);
    msg.torqueLoopVelocityLimit.resize(1);

    msg.name[0]                         = "/r2/right_leg/gripper/joint0";
    msg.desiredPosition[0]              = 1;
    msg.desiredPositionVelocityLimit[0] = 2;
    msg.feedForwardTorque[0]            = 3;
    msg.proportionalGain[0]             = 4;
    msg.derivativeGain[0]               = 5;
    msg.integralGain[0]                 = 6;
    msg.positionLoopTorqueLimit[0]      = 7;
    msg.positionLoopWindupLimit[0]      = 8;
    msg.torqueLoopVelocityLimit[0]      = 9;

    gripperJoint.setCommand(msg, controlMsg);

    EXPECT_FLOAT_EQ(1, getFloat("/r2/right_leg/gripper/joint0/PosCom"));
    EXPECT_FLOAT_EQ(2, getFloat("/r2/right_leg/gripper/joint0/PosComVelocity"));
    EXPECT_FLOAT_EQ(3, getFloat("/r2/right_leg/gripper/joint0/TorqueCom"));
    EXPECT_FLOAT_EQ(4, getFloat("/r2/right_leg/gripper/joint0/JointStiffness"));
    EXPECT_FLOAT_EQ(5, getFloat("/r2/right_leg/gripper/joint0/JointDamping"));
    EXPECT_FLOAT_EQ(6, getFloat("/r2/right_leg/gripper/joint0/PLKi"));
    EXPECT_FLOAT_EQ(7, getFloat("/r2/right_leg/gripper/joint0/TorqueLim"));
    EXPECT_FLOAT_EQ(8, getFloat("/r2/right_leg/gripper/joint0/PLWindupLim"));
    //EXPECT_FLOAT_EQ(9, getFloat("/right_leg/joint7/TLVelLimit"));

    msg.desiredPositionVelocityLimit.resize(0);
    msg.proportionalGain.resize(0);
    msg.integralGain.resize(0);
    msg.positionLoopWindupLimit.resize(0);

    msg.desiredPosition[0]         = 10;
    msg.feedForwardTorque[0]       = 30;
    msg.derivativeGain[0]          = 50;
    msg.positionLoopTorqueLimit[0] = 70;
    msg.torqueLoopVelocityLimit[0] = 90;

    gripperJoint.setCommand(msg, controlMsg);

    EXPECT_FLOAT_EQ(10, getFloat("/r2/right_leg/gripper/joint0/PosCom"));
    EXPECT_FLOAT_EQ(2,  getFloat("/r2/right_leg/gripper/joint0/PosComVelocity"));
    EXPECT_FLOAT_EQ(30, getFloat("/r2/right_leg/gripper/joint0/TorqueCom"));
    EXPECT_FLOAT_EQ(4,  getFloat("/r2/right_leg/gripper/joint0/JointStiffness"));
    EXPECT_FLOAT_EQ(50, getFloat("/r2/right_leg/gripper/joint0/JointDamping"));
    EXPECT_FLOAT_EQ(6,  getFloat("/r2/right_leg/gripper/joint0/PLKi"));
    EXPECT_FLOAT_EQ(70, getFloat("/r2/right_leg/gripper/joint0/TorqueLim"));
    EXPECT_FLOAT_EQ(8,  getFloat("/r2/right_leg/gripper/joint0/PLWindupLim"));
    //EXPECT_FLOAT_EQ(90, getFloat("/right_leg/joint7/TLVelLimit"));
}

TEST_F(JointCommandGripperTest, setCommand_BadMsg)
{
    JointCommandGripper gripperJoint("/r2/right_leg/gripper/joint0", io);

    nasa_r2_common_msgs::JointCommand          msg;
    nasa_r2_common_msgs::JointControlData controlMsg;
    controlMsg.controlMode.state = nasa_r2_common_msgs::JointControlMode::PARK;

    //! empty command message
    EXPECT_THROW(gripperJoint.setCommand(msg, controlMsg), std::runtime_error);

    msg.header.stamp = ros::Time::now();

    msg.name.resize(1);
    msg.desiredPosition.resize(1);
    msg.desiredPositionVelocityLimit.resize(1);
    msg.feedForwardTorque.resize(1);
    msg.proportionalGain.resize(1);
    msg.derivativeGain.resize(1);
    msg.integralGain.resize(1);
    msg.positionLoopTorqueLimit.resize(1);
    msg.positionLoopWindupLimit.resize(1);
    msg.torqueLoopVelocityLimit.resize(1);

    msg.name[0]                         = "/r2/right_leg/gripper/joint1";
    msg.desiredPosition[0]              = 1;
    msg.desiredPositionVelocityLimit[0] = 2;
    msg.feedForwardTorque[0]            = 3;
    msg.proportionalGain[0]             = 4;
    msg.derivativeGain[0]               = 5;
    msg.integralGain[0]                 = 6;
    msg.positionLoopTorqueLimit[0]      = 7;
    msg.positionLoopWindupLimit[0]      = 8;
    msg.torqueLoopVelocityLimit[0]      = 9;

    //! wrong joint name
    EXPECT_THROW(gripperJoint.setCommand(msg, controlMsg), std::runtime_error);

    msg.name[0] = "/r2/right_leg/gripper/joint0";
    msg.desiredPosition.push_back(11);

    //! too many desiredPosition values
    EXPECT_THROW(gripperJoint.setCommand(msg, controlMsg), std::runtime_error);
}

TEST_F(JointCommandGripperTest, getCapability)
{
    JointCommandGripper gripperJoint("/r2/right_leg/gripper/joint0", io);

    nasa_r2_common_msgs::JointCapability msg = gripperJoint.getCapability();

    EXPECT_EQ(1, msg.name.size());
    EXPECT_EQ(1, msg.positionLimitMax.size());
    EXPECT_EQ(1, msg.positionLimitMin.size());
    EXPECT_EQ(1, msg.torqueLimit.size());

    EXPECT_STREQ("/r2/right_leg/gripper/joint0", msg.name[0].c_str());
    EXPECT_FLOAT_EQ(3.14159265358979, msg.positionLimitMax[0]);
    EXPECT_FLOAT_EQ(-3.14159265358979, msg.positionLimitMin[0]);
    EXPECT_FLOAT_EQ(0., msg.torqueLimit[0]);
}

int main(int argc, char **argv){
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
