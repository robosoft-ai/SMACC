#include <gtest/gtest.h>
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include "nasa_r2_config_core/RobotInstanceFactory.h"
#include "nasa_r2_config_core/CoeffMapLoader.h"
#include "nasa_robodyn_mechanisms_core/JointCommandRigid.h"
#include <ros/package.h>

using namespace std;

std::map<std::string, float>    testFloats;
std::map<std::string, uint32_t> testUInt32s;
std::map<std::string, uint16_t> testUInt16s;
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

float getUInt16(std::string item)
{
    return testUInt16s[item];
}

void setUInt16(std::string item, uint16_t value)
{
    testUInt16s[item] = value;
}

uint32_t getUInt32(std::string item)
{
    return testUInt32s[item];
}

void setUInt32(std::string item, uint32_t value)
{
    testUInt32s[item] = value;
}

float getMotorCoeff(std::string item)
{
    return motorCoeffMap->operator[](item);
}

bool hasBrainstemCoeff(std::string item)
{
    if (brainstemCoeffMap->find(item) != brainstemCoeffMap->end())
    {
        return true;
    }

    return false;
}

float getBrainstemCoeff(std::string item)
{
    return brainstemCoeffMap->operator[](item);
}

std::string getCommandFile(std::string mechanism)
{
    return instance->configurationSafetyBasePath + instance->APIMAP_PATH + instance->mechanismCommandMap[mechanism];
}

class JointCommandRigidTest : public ::testing::Test {
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
            testUInt32s.clear();

            io.getFloat          = getFloat;
            io.setFloat          = setFloat;
            io.setUInt16         = setUInt16;
            io.getUInt32         = getUInt32;
            io.getMotorCoeff     = getMotorCoeff;
            io.hasBrainstemCoeff = hasBrainstemCoeff;
            io.getBrainstemCoeff = getBrainstemCoeff;
            io.getCommandFile    = getCommandFile;

            ros::Time::init();
        }

        virtual void TearDown()
        {
        }

        string packagePath, configPath, configSafetyPath, fileName;
        JointCommandInterface::IoFunctions io;
};

TEST_F(JointCommandRigidTest, Constructor)
{
    ASSERT_NO_THROW(JointCommandRigid rigidJoint("/r2/left_leg/joint0", io));

    //! Missing IO function definitions
    io.getFloat = 0;
    ASSERT_THROW(JointCommandRigid rigidJoint("/r2/left_leg/joint0", io), std::invalid_argument);
}

TEST_F(JointCommandRigidTest, LoadCoeffs)
{
    JointCommandRigid rigidJoint("/r2/left_leg/joint0", io);

    EXPECT_NO_THROW(rigidJoint.loadCoeffs());

    nasa_r2_common_msgs::JointControlData controlMsg;
    controlMsg.coeffState.state = nasa_r2_common_msgs::JointControlCoeffState::LOADED;
    EXPECT_NO_THROW(rigidJoint.updateMeasuredState(controlMsg));

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

    msg.name[0]                         = "/r2/left_leg/joint0";
    msg.desiredPosition[0]              = 1;
    msg.desiredPositionVelocityLimit[0] = 2;
    msg.feedForwardTorque[0]            = 3;
    msg.proportionalGain[0]             = 4;
    msg.derivativeGain[0]               = 5;
    msg.integralGain[0]                 = 6;
    msg.positionLoopTorqueLimit[0]      = 7;
    msg.positionLoopWindupLimit[0]      = 8;
    msg.torqueLoopVelocityLimit[0]      = 9;

    EXPECT_NO_THROW(rigidJoint.setCommand(msg, controlMsg));

    EXPECT_NO_THROW(rigidJoint.getCommandedState());

    EXPECT_NO_THROW(rigidJoint.getCapability());
}

TEST_F(JointCommandRigidTest, getSimpleMeasuredState)
{
    JointCommandRigid rigidJoint("/r2/left_leg/joint0", io);

    setFloat("/r2/left_leg/joint0/APS2Unfiltered", 1);
    setFloat("/r2/left_leg/joint0/JointVelocity", 2);
    setFloat("/r2/left_leg/joint0/EncPos", 5);
    setFloat("/r2/left_leg/joint0/EncVel", 6);

    nasa_r2_common_msgs::JointControlData controlMsg;
    controlMsg.coeffState.state = nasa_r2_common_msgs::JointControlCoeffState::LOADED;
    rigidJoint.updateMeasuredState(controlMsg);
    sensor_msgs::JointState msg = rigidJoint.getSimpleMeasuredState();

    EXPECT_EQ(1, msg.name.size());
    EXPECT_EQ(1, msg.position.size());
    EXPECT_EQ(1, msg.velocity.size());
    EXPECT_EQ(1, msg.effort.size());

    EXPECT_STREQ("/r2/left_leg/joint0", msg.name[0].c_str());
    EXPECT_FLOAT_EQ(1,                  msg.position[0]);
    EXPECT_FLOAT_EQ(2,                  msg.velocity[0]);
    EXPECT_FLOAT_EQ(0,                  msg.effort[0]);
}

TEST_F(JointCommandRigidTest, getCommandedState)
{
    JointCommandRigid rigidJoint("/r2/left_leg/joint0", io);

    setFloat("/r2/left_leg/joint0/PosCom", 1);
    setFloat("/r2/left_leg/joint0/PosComVelocity", 2);

    nasa_r2_common_msgs::JointCommand msg = rigidJoint.getCommandedState();

    EXPECT_EQ(1, msg.name.size());
    EXPECT_EQ(1, msg.desiredPosition.size());
    EXPECT_EQ(1, msg.desiredPositionVelocityLimit.size());
    EXPECT_EQ(1, msg.feedForwardTorque.size());
    EXPECT_EQ(1, msg.proportionalGain.size());
    EXPECT_EQ(1, msg.derivativeGain.size());
    EXPECT_EQ(1, msg.integralGain.size());
    EXPECT_EQ(1, msg.positionLoopTorqueLimit.size());
    EXPECT_EQ(1, msg.positionLoopWindupLimit.size());
    EXPECT_EQ(1, msg.torqueLoopVelocityLimit.size());

    EXPECT_STREQ("/r2/left_leg/joint0", msg.name[0].c_str());
    EXPECT_FLOAT_EQ(1, msg.desiredPosition[0]);
    EXPECT_FLOAT_EQ(2, msg.desiredPositionVelocityLimit[0]);
    EXPECT_FLOAT_EQ(0, msg.feedForwardTorque[0]);
    EXPECT_FLOAT_EQ(0, msg.proportionalGain[0]);
    EXPECT_FLOAT_EQ(0, msg.derivativeGain[0]);
    EXPECT_FLOAT_EQ(0, msg.integralGain[0]);
    EXPECT_FLOAT_EQ(0, msg.positionLoopTorqueLimit[0]);
    EXPECT_FLOAT_EQ(0, msg.positionLoopWindupLimit[0]);
    EXPECT_FLOAT_EQ(0, msg.torqueLoopVelocityLimit[0]);
}

TEST_F(JointCommandRigidTest, setCommand)
{
    JointCommandRigid rigidJoint("/r2/left_leg/joint0", io);

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

    msg.name[0]                         = "/r2/left_leg/joint0";
    msg.desiredPosition[0]              = 1;
    msg.desiredPositionVelocityLimit[0] = 2;

    rigidJoint.setCommand(msg, controlMsg);

    EXPECT_FLOAT_EQ(1, getFloat("/r2/left_leg/joint0/PosCom"));
    EXPECT_FLOAT_EQ(2, getFloat("/r2/left_leg/joint0/PosComVelocity"));
    EXPECT_FLOAT_EQ(0, getFloat("/r2/left_leg/joint0/TorqueCom"));
    EXPECT_FLOAT_EQ(0, getFloat("/r2/left_leg/joint0/JointStiffness"));
    EXPECT_FLOAT_EQ(0, getFloat("/r2/left_leg/joint0/JointDamping"));
    EXPECT_FLOAT_EQ(0, getFloat("/r2/left_leg/joint0/PLKi"));
    EXPECT_FLOAT_EQ(0, getFloat("/r2/left_leg/joint0/TorqueLim"));
    EXPECT_FLOAT_EQ(0, getFloat("/r2/left_leg/joint0/PLWindupLim"));
    EXPECT_FLOAT_EQ(0, getFloat("/left_leg/joint0/TLVelLimit"));

    msg.desiredPositionVelocityLimit.resize(0);
    msg.proportionalGain.resize(0);
    msg.integralGain.resize(0);
    msg.positionLoopWindupLimit.resize(0);

    msg.desiredPosition[0]         = 10;

    rigidJoint.setCommand(msg, controlMsg);

    EXPECT_FLOAT_EQ(10, getFloat("/r2/left_leg/joint0/PosCom"));
    EXPECT_FLOAT_EQ(2,  getFloat("/r2/left_leg/joint0/PosComVelocity"));
    EXPECT_FLOAT_EQ(0,  getFloat("/r2/left_leg/joint0/TorqueCom"));
    EXPECT_FLOAT_EQ(0,  getFloat("/r2/left_leg/joint0/JointStiffness"));
    EXPECT_FLOAT_EQ(0,  getFloat("/r2/left_leg/joint0/JointDamping"));
    EXPECT_FLOAT_EQ(0,  getFloat("/r2/left_leg/joint0/PLKi"));
    EXPECT_FLOAT_EQ(0,  getFloat("/r2/left_leg/joint0/TorqueLim"));
    EXPECT_FLOAT_EQ(0,  getFloat("/r2/left_leg/joint0/PLWindupLim"));
    EXPECT_FLOAT_EQ(0,  getFloat("/right_leg/joint7/TLVelLimit"));    
}

TEST_F(JointCommandRigidTest, setCommand_BadMsg)
{
    JointCommandRigid rigidJoint("/r2/left_leg/joint0", io);

    nasa_r2_common_msgs::JointCommand          msg;
    nasa_r2_common_msgs::JointControlData controlMsg;
    controlMsg.controlMode.state = nasa_r2_common_msgs::JointControlMode::PARK;

    //! empty command message
    EXPECT_THROW(rigidJoint.setCommand(msg, controlMsg), std::runtime_error);

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

    msg.name[0]                         = "/r2/left_leg/joint1";
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
    EXPECT_THROW(rigidJoint.setCommand(msg, controlMsg), std::runtime_error);

    msg.name[0] = "/r2/left_leg/joint0";
    msg.desiredPosition.push_back(11);

    //! too many desiredPosition values
    EXPECT_THROW(rigidJoint.setCommand(msg, controlMsg), std::runtime_error);
}

TEST_F(JointCommandRigidTest, getCapability)
{
    JointCommandRigid rigidJoint("/r2/left_leg/joint0", io);

    nasa_r2_common_msgs::JointCapability msg = rigidJoint.getCapability();

    EXPECT_EQ(1, msg.name.size());
    EXPECT_EQ(1, msg.positionLimitMax.size());
    EXPECT_EQ(1, msg.positionLimitMin.size());
    EXPECT_EQ(1, msg.torqueLimit.size());

    EXPECT_STREQ("/r2/left_leg/joint0", msg.name[0].c_str());
    EXPECT_FLOAT_EQ(3.14159265358979, msg.positionLimitMax[0]);
    EXPECT_FLOAT_EQ(-3.14159265358979, msg.positionLimitMin[0]);
    EXPECT_FLOAT_EQ(0, msg.torqueLimit[0]);
}

int main(int argc, char **argv){
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
