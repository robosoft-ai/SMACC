#include <gtest/gtest.h>
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include "nasa_r2_config_core/RobotInstanceFactory.h"
#include "nasa_r2_config_core/CoeffMapLoader.h"
#include "nasa_robodyn_mechanisms_core/JointCommandSeriesElastic.h"
#include <ros/package.h>

using namespace std;

std::map<std::string, float>    testFloats;
std::map<std::string, int32_t>  testInt32s;
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

int32_t getInt32(std::string item)
{
    return testInt32s[item];
}

void setInt32(std::string item, int32_t value)
{
    testInt32s[item] = value;
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

class JointCommandSeriesElasticTest : public ::testing::Test {
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
            testInt32s.clear();

            io.getFloat          = getFloat;
            io.setFloat          = setFloat;
            io.setUInt16         = setUInt16;
            io.getInt32          = getInt32;
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

TEST_F(JointCommandSeriesElasticTest, Constructor)
{
    ASSERT_NO_THROW(JointCommandSeriesElastic seriesElasticJoint("/r2/left_leg/joint0", io));

    //! Missing IO function definitions
    io.getFloat = 0;
    ASSERT_THROW(JointCommandSeriesElastic seriesElasticJoint("/r2/left_leg/joint0", io), std::invalid_argument);
}

TEST_F(JointCommandSeriesElasticTest, LoadCoeffs)
{
    JointCommandSeriesElastic seriesElasticJoint("/r2/left_leg/joint0", io);

    EXPECT_NO_THROW(seriesElasticJoint.loadCoeffs());

    nasa_r2_common_msgs::JointControlData controlMsg;
    controlMsg.coeffState.state = nasa_r2_common_msgs::JointControlCoeffState::LOADED;
    EXPECT_NO_THROW(seriesElasticJoint.updateMeasuredState(controlMsg));

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

    EXPECT_NO_THROW(seriesElasticJoint.setCommand(msg, controlMsg));

    EXPECT_NO_THROW(seriesElasticJoint.getCommandedState());

    EXPECT_NO_THROW(seriesElasticJoint.getCapability());
}

TEST_F(JointCommandSeriesElasticTest, getSimpleMeasuredState)
{
    JointCommandSeriesElastic seriesElasticJoint("/r2/left_leg/joint0", io);

    setFloat("/r2/left_leg/joint0/APS2Unfiltered", 1);
    setFloat("/r2/left_leg/joint0/JointVelocity", 2);
    setFloat("/r2/left_leg/joint0/APS1Unfiltered", 3);
    setFloat("/r2/left_leg/joint0/APS1Vel", 4);
    setFloat("/r2/left_leg/joint0/EncPos", 5);
    setFloat("/r2/left_leg/joint0/EncVel", 6);
    setInt32("/r2/left_leg/joint0/IncEnc", 123);
    setFloat("/r2/left_leg/joint0/JointTorque", 10);

    nasa_r2_common_msgs::JointControlData controlMsg;
    controlMsg.coeffState.state = nasa_r2_common_msgs::JointControlCoeffState::LOADED;
    seriesElasticJoint.updateMeasuredState(controlMsg);
    sensor_msgs::JointState msg = seriesElasticJoint.getSimpleMeasuredState();

    EXPECT_EQ(1, msg.name.size());
    EXPECT_EQ(1, msg.position.size());
    EXPECT_EQ(1, msg.velocity.size());
    EXPECT_EQ(1, msg.effort.size());

    EXPECT_STREQ("/r2/left_leg/joint0", msg.name[0].c_str());
    EXPECT_FLOAT_EQ(1,                  msg.position[0]);
    EXPECT_FLOAT_EQ(2,                  msg.velocity[0]);
    EXPECT_FLOAT_EQ(-10,                 msg.effort[0]);
}

TEST_F(JointCommandSeriesElasticTest, getCompleteMeasuredState)
{
    JointCommandSeriesElastic seriesElasticJoint("/r2/left_leg/joint0", io);

    setFloat("/r2/left_leg/joint0/APS2Unfiltered", 1);
    setFloat("/r2/left_leg/joint0/JointVelocity", 2);
    setFloat("/r2/left_leg/joint0/APS1Unfiltered", 3);
    setFloat("/r2/left_leg/joint0/APS1Vel", 4);
    setFloat("/r2/left_leg/joint0/EncPos", 5);
    setFloat("/r2/left_leg/joint0/EncVel", 6);
    setFloat("/r2/left_leg/joint0/APS2", 1.1);
    setFloat("/r2/left_leg/joint0/APS1", 3.1);
    setInt32("/r2/left_leg/joint0/IncEnc", 123);
    setFloat("/r2/left_leg/joint0/JointTorque", 10);
    setFloat("/r2/left_leg/joint0/PosComSmoothed", 12);
    setFloat("/r2/left_leg/joint0/VelSmoothedFiltered", 13);
    setFloat("/r2/left_leg/joint0/TorqueDes", 14);

    nasa_r2_common_msgs::JointControlData controlMsg;
    controlMsg.coeffState.state = nasa_r2_common_msgs::JointControlCoeffState::LOADED;
    seriesElasticJoint.updateMeasuredState(controlMsg);
    sensor_msgs::JointState msg = seriesElasticJoint.getCompleteMeasuredState();

    EXPECT_EQ(7, msg.name.size());
    EXPECT_EQ(7, msg.position.size());
    EXPECT_EQ(7, msg.velocity.size());
    EXPECT_EQ(7, msg.effort.size());

    EXPECT_STREQ("/r2/left_leg/joint0",             msg.name[0].c_str());
    EXPECT_STREQ("/r2/left_leg/motor0",             msg.name[1].c_str());
    EXPECT_STREQ("/r2/left_leg/encoder0",           msg.name[2].c_str());
    EXPECT_STREQ("/r2/left_leg/jointCalculated0",   msg.name[3].c_str());
    EXPECT_STREQ("/r2/left_leg/encoderCalculated0", msg.name[4].c_str());
    EXPECT_STREQ("/r2/left_leg/hallsCalculated0",   msg.name[5].c_str());
    EXPECT_STREQ("/r2/left_leg/embeddedCommand0",   msg.name[6].c_str());

    //! joint
    EXPECT_FLOAT_EQ(1,  msg.position[0]);
    EXPECT_FLOAT_EQ(2,  msg.velocity[0]);
    EXPECT_FLOAT_EQ(-10, msg.effort[0]);

    //! motor
    EXPECT_FLOAT_EQ(3, msg.position[1]);
    EXPECT_FLOAT_EQ(4, msg.velocity[1]);
    //EXPECT_FLOAT_EQ(0, msg.effort[1]);  //! needs some calculation

    //! encoder
    EXPECT_FLOAT_EQ(5,           msg.position[2]);
    EXPECT_FLOAT_EQ(6.0 / 160.0, msg.velocity[2]);
    EXPECT_FLOAT_EQ(0,           msg.effort[2]);

    //! joint calculated
    EXPECT_FLOAT_EQ(0,             msg.position[3]);
    EXPECT_FLOAT_EQ(0,             msg.velocity[3]);
    EXPECT_FLOAT_EQ((1.1 - 3.1) * 100.0, msg.effort[3]);

    //! encoder calculated
    //EXPECT_FLOAT_EQ(0, msg.position[4]);  //! needs some calculation
    //EXPECT_FLOAT_EQ(0, msg.velocity[4]);  //! needs some calculation
    EXPECT_FLOAT_EQ(0, msg.effort[4]);

    //! halls calculated
    //EXPECT_FLOAT_EQ(0, msg.position[5]);  //! needs some calculation
    //EXPECT_FLOAT_EQ(0, msg.velocity[5]);  //! needs some calculation
    EXPECT_FLOAT_EQ(0, msg.effort[5]);

    //! embedded command
    EXPECT_FLOAT_EQ(12, msg.position[6]);
    EXPECT_FLOAT_EQ(13, msg.velocity[6]);
    EXPECT_FLOAT_EQ(14, msg.effort[6]);
}

TEST_F(JointCommandSeriesElasticTest, getCommandedState)
{
    JointCommandSeriesElastic seriesElasticJoint("/r2/left_leg/joint0", io);

    setFloat("/r2/left_leg/joint0/PosCom", 1);
    setFloat("/r2/left_leg/joint0/PosComVelocity", 2);
    setFloat("/r2/left_leg/joint0/TorqueCom", 3);
    setFloat("/r2/left_leg/joint0/JointStiffness", 4);
    setFloat("/r2/left_leg/joint0/JointDamping", 5);
    setFloat("/r2/left_leg/joint0/PLKi", 6);
    setFloat("/r2/left_leg/joint0/TorqueLim", 7);
    setFloat("/r2/left_leg/joint0/PLWindupLim", 8);
    setFloat("/r2/left_leg/joint0/TLVelLimit", 9);

    nasa_r2_common_msgs::JointCommand msg = seriesElasticJoint.getCommandedState();

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
    EXPECT_FLOAT_EQ(3, msg.feedForwardTorque[0]);
    EXPECT_FLOAT_EQ(4, msg.proportionalGain[0]);
    EXPECT_FLOAT_EQ(5, msg.derivativeGain[0]);
    EXPECT_FLOAT_EQ(6, msg.integralGain[0]);
    EXPECT_FLOAT_EQ(7, msg.positionLoopTorqueLimit[0]);
    EXPECT_FLOAT_EQ(8, msg.positionLoopWindupLimit[0]);
    // EXPECT_FLOAT_EQ(9, msg.torqueLoopVelocityLimit[0]);
}

TEST_F(JointCommandSeriesElasticTest, setCommand)
{
    JointCommandSeriesElastic seriesElasticJoint("/r2/left_leg/joint0", io);

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
    msg.feedForwardTorque[0]            = 3;
    msg.proportionalGain[0]             = 4;
    msg.derivativeGain[0]               = 5;
    msg.integralGain[0]                 = 6;
    msg.positionLoopTorqueLimit[0]      = 7;
    msg.positionLoopWindupLimit[0]      = 8;
    msg.torqueLoopVelocityLimit[0]      = 9;

    seriesElasticJoint.setCommand(msg, controlMsg);

    EXPECT_FLOAT_EQ(1, getFloat("/r2/left_leg/joint0/PosCom"));
    EXPECT_FLOAT_EQ(2, getFloat("/r2/left_leg/joint0/PosComVelocity"));
    EXPECT_FLOAT_EQ(3, getFloat("/r2/left_leg/joint0/TorqueCom"));
    EXPECT_FLOAT_EQ(4, getFloat("/r2/left_leg/joint0/JointStiffness"));
    EXPECT_FLOAT_EQ(5, getFloat("/r2/left_leg/joint0/JointDamping"));
    EXPECT_FLOAT_EQ(6, getFloat("/r2/left_leg/joint0/PLKi"));
    EXPECT_FLOAT_EQ(7, getFloat("/r2/left_leg/joint0/TorqueLim"));
    EXPECT_FLOAT_EQ(8, getFloat("/r2/left_leg/joint0/PLWindupLim"));
    //EXPECT_FLOAT_EQ(9, getFloat("/left_leg/joint0/TLVelLimit"));

    msg.desiredPositionVelocityLimit.resize(0);
    msg.proportionalGain.resize(0);
    msg.integralGain.resize(0);
    msg.positionLoopWindupLimit.resize(0);

    msg.desiredPosition[0]         = 10;
    msg.feedForwardTorque[0]       = 30;
    msg.derivativeGain[0]          = 50;
    msg.positionLoopTorqueLimit[0] = 70;
    msg.torqueLoopVelocityLimit[0] = 90;

    seriesElasticJoint.setCommand(msg, controlMsg);

    EXPECT_FLOAT_EQ(10, getFloat("/r2/left_leg/joint0/PosCom"));
    EXPECT_FLOAT_EQ(2,  getFloat("/r2/left_leg/joint0/PosComVelocity"));
    EXPECT_FLOAT_EQ(30, getFloat("/r2/left_leg/joint0/TorqueCom"));
    EXPECT_FLOAT_EQ(4,  getFloat("/r2/left_leg/joint0/JointStiffness"));
    EXPECT_FLOAT_EQ(50, getFloat("/r2/left_leg/joint0/JointDamping"));
    EXPECT_FLOAT_EQ(6,  getFloat("/r2/left_leg/joint0/PLKi"));
    EXPECT_FLOAT_EQ(70, getFloat("/r2/left_leg/joint0/TorqueLim"));
    EXPECT_FLOAT_EQ(8,  getFloat("/r2/left_leg/joint0/PLWindupLim"));
    //EXPECT_FLOAT_EQ(90, getFloat("/right_leg/joint7/TLVelLimit"));    
}

TEST_F(JointCommandSeriesElasticTest, setCommand_BadMsg)
{
    JointCommandSeriesElastic seriesElasticJoint("/r2/left_leg/joint0", io);

    nasa_r2_common_msgs::JointCommand          msg;
    nasa_r2_common_msgs::JointControlData controlMsg;
    controlMsg.controlMode.state = nasa_r2_common_msgs::JointControlMode::PARK;

    //! empty command message
    EXPECT_THROW(seriesElasticJoint.setCommand(msg, controlMsg), std::runtime_error);

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
    EXPECT_THROW(seriesElasticJoint.setCommand(msg, controlMsg), std::runtime_error);

    msg.name[0] = "/r2/left_leg/joint0";
    msg.desiredPosition.push_back(11);

    //! too many desiredPosition values
    EXPECT_THROW(seriesElasticJoint.setCommand(msg, controlMsg), std::runtime_error);
}

TEST_F(JointCommandSeriesElasticTest, getCapability)
{
    JointCommandSeriesElastic seriesElasticJoint("/r2/left_leg/joint0", io);

    nasa_r2_common_msgs::JointCapability msg = seriesElasticJoint.getCapability();

    EXPECT_EQ(1, msg.name.size());
    EXPECT_EQ(1, msg.positionLimitMax.size());
    EXPECT_EQ(1, msg.positionLimitMin.size());
    EXPECT_EQ(1, msg.torqueLimit.size());

    EXPECT_STREQ("/r2/left_leg/joint0", msg.name[0].c_str());
    EXPECT_FLOAT_EQ(3.14159265358979, msg.positionLimitMax[0]);
    EXPECT_FLOAT_EQ(-3.14159265358979, msg.positionLimitMin[0]);
    EXPECT_FLOAT_EQ(271.1, msg.torqueLimit[0]);
}

int main(int argc, char **argv){
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
