#include <gtest/gtest.h>
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include "nasa_r2_config_core/RobotInstanceFactory.h"
#include "nasa_r2_config_core/CoeffMapLoader.h"
#include "nasa_robodyn_mechanisms_core/JointCommandWrist.h"
#include <ros/package.h>
#include "nasa_r2_common_msgs/JointControlData.h"

#define TEST_BED

using namespace std;
using namespace Eigen;

std::map<std::string, int16_t> testInt16;
std::map<std::string, uint16_t> testUint16;
std::map<std::string, float> testLiveCoeffs;
CoeffMapPtr motorCoeffMap;
CoeffMapPtr brainstemCoeffMap;
RobotInstancePtr instance;


float getInt16(std::string item)
{
    return testInt16[item];
}

float getUint16(std::string item)
{
    return testUint16[item];
}

void setInt16(std::string item, int16_t value)
{
    testInt16[item] = value;
}

void setUint16(std::string item, uint16_t value)
{
    testUint16[item] = value;
}

float getBrainstemCoeff(std::string item)
{
    return brainstemCoeffMap->operator[](item);
}

float getMotorCoeff(std::string item)
{
    return motorCoeffMap->operator[](item);
}

bool hasLiveCoeff(std::string coeff)
{
    return testLiveCoeffs.count(coeff) > 0;
}

float getLiveCoeff(std::string coeff)
{
    return testLiveCoeffs[coeff];
}

void setLiveCoeff(std::string coeff, float value)
{
    testLiveCoeffs[coeff] = value;
}

std::vector<std::string> getJointNames(std::string mechanism)
{
    return instance->mechanismJointsMap[mechanism];
}

std::vector<std::string> getActuatorNames(std::string mechanism)
{
    return instance->mechanismActuatorsMap[mechanism];
}

std::string getCommandFile(std::string mechanism)
{
    return instance->configurationSafetyBasePath + instance->APIMAP_PATH + instance->mechanismCommandMap[mechanism];
}

class JointCommandWristTest : public ::testing::Test {
    protected:
        virtual void SetUp()
        {
            packagePath      = ros::package::getPath("nasa_r2_config_core");
            configPath       = packagePath + "/test/config";
            configSafetyPath = packagePath + "/test/configSafety";
            fileName         = "WristTestRobotInstance.xml";
            instance         = RobotInstanceFactory::createRobotInstanceFromFile(configPath, configSafetyPath, fileName);

            motorCoeffMap = boost::make_shared<CoeffMap>();
            brainstemCoeffMap = boost::make_shared<CoeffMap>();
            CoeffMapLoader::loadElements(instance, motorCoeffMap, brainstemCoeffMap);

            testInt16.clear();
            testUint16.clear();
            testLiveCoeffs.clear();

            io.setInt16         = setInt16;
            io.getInt16         = getInt16;
            io.getUInt16        = getUint16;
            io.getBrainstemCoeff= getBrainstemCoeff;
            io.getMotorCoeff    = getMotorCoeff;
            io.getJointNames    = getJointNames;
            io.getActuatorNames = getActuatorNames;
            io.getCommandFile   = getCommandFile;
            io.hasLiveCoeff   = hasLiveCoeff;
            io.getLiveCoeff   = getLiveCoeff;
            io.setLiveCoeff   = setLiveCoeff;

            ros::Time::init();
            mechanism = "/r2/right_arm/wrist";
            calEncoder = Vector2f(0,0);
            calAngle(0) = io.getMotorCoeff("/r2/right_arm/wrist/ManualCalPitch");
            calAngle(1) = io.getMotorCoeff("/r2/right_arm/wrist/ManualCalYaw");
            calSlider(0) = io.getMotorCoeff("/r2/right_arm/wrist/ManualCalThumbSlider");
            calSlider(1) = io.getMotorCoeff("/r2/right_arm/wrist/ManualCalLittleSlider");
            calibrationModeName = "/r2/right_arm/wrist/CalibrationMode";
            io.setLiveCoeff(calibrationModeName, nasa_r2_common_msgs::JointControlCalibrationMode::IGNORE);
            controlMsg.coeffState.state = nasa_r2_common_msgs::JointControlCoeffState::LOADED;
        }

        virtual void TearDown()
        {
        }

        string packagePath, configPath, configSafetyPath, fileName;
        string mechanism;
        JointCommandInterface::IoFunctions io;
        Vector2f calEncoder, calAngle, calSlider;
        nasa_r2_common_msgs::JointControlData controlMsg;
        nasa_r2_common_msgs::JointCommand commandMsg;
        string calibrationModeName;
};

TEST_F(JointCommandWristTest, Constructor)
{
    ASSERT_NO_THROW(JointCommandWrist wristJoint(mechanism, io));
    //! Missing IO function definitions
    io.getInt16 = 0;
    ASSERT_THROW(JointCommandWrist wristJoint(mechanism, io), std::invalid_argument);
}

TEST_F(JointCommandWristTest, LoadCoeffs)
{
    JointCommandWrist wristJoint(mechanism, io);

    EXPECT_NO_THROW(wristJoint.loadCoeffs());

    nasa_r2_common_msgs::JointControlData controlMsg;
    controlMsg.coeffState.state = nasa_r2_common_msgs::JointControlCoeffState::LOADED;
    EXPECT_NO_THROW(wristJoint.updateMeasuredState(controlMsg));

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

    msg.name[0]                         = mechanism;
    msg.desiredPosition[0]              = 1;
    msg.desiredPositionVelocityLimit[0] = 2;
    msg.feedForwardTorque[0]            = 3;
    msg.proportionalGain[0]             = 4;
    msg.derivativeGain[0]               = 5;
    msg.integralGain[0]                 = 6;
    msg.positionLoopTorqueLimit[0]      = 7;
    msg.positionLoopWindupLimit[0]      = 8;
    msg.torqueLoopVelocityLimit[0]      = 9;

    EXPECT_NO_THROW(wristJoint.setCommand(msg, controlMsg));

    EXPECT_NO_THROW(wristJoint.getCommandedState());

    EXPECT_NO_THROW(wristJoint.getCapability());
}

//! @todo update this for halls check once calibrated
TEST_F(JointCommandWristTest, getSimpleMeasuredState)
{
    JointCommandWrist wristJoint(mechanism, io);

    io.setLiveCoeff("/r2/right_arm/wrist/UseHalls", 0);

    setInt16("/r2/right_arm/wrist/thumbside/IncEnc", calEncoder(0));
    setInt16("/r2/right_arm/wrist/thumbside/MotCom", 5);
    setInt16("/r2/right_arm/wrist/littleside/IncEnc", calEncoder(1));
    setInt16("/r2/right_arm/wrist/littleside/MotCom", 5);
//    setUint16("/r2/right_arm/wrist/pitch/Hall", calAngle(0));
//    setUint16("/r2/right_arm/wrist/yaw/Hall", calAngle(1));

    wristJoint.updateMeasuredState(controlMsg);
    sensor_msgs::JointState msg = wristJoint.getSimpleMeasuredState();

    EXPECT_EQ(4, msg.name.size());
    EXPECT_EQ(4, msg.position.size());
    EXPECT_EQ(4, msg.velocity.size());
    EXPECT_EQ(4, msg.effort.size());

    EXPECT_STREQ("/r2/right_arm/wrist/pitch",               msg.name[0].c_str());
    EXPECT_STREQ("/r2/right_arm/wrist/yaw",                 msg.name[1].c_str());
    //EXPECT_STREQ("/r2/right_arm/wrist/thumbside/encoder",   msg.name[2].c_str());
    //EXPECT_STREQ("/r2/right_arm/wrist/littleside/encoder",  msg.name[3].c_str());
    EXPECT_STREQ("/r2/right_arm/wrist/thumbside",    msg.name[2].c_str());
    EXPECT_STREQ("/r2/right_arm/wrist/littleside",   msg.name[3].c_str());
    //EXPECT_STREQ("/r2/right_arm/wrist/pitch/halls",         msg.name[6].c_str());
    //EXPECT_STREQ("/r2/right_arm/wrist/yaw/halls",           msg.name[7].c_str());

    EXPECT_NEAR(0, msg.position[0], 0.00174532925);
    EXPECT_NEAR(0, msg.position[1], 0.00174532925);
    //EXPECT_FLOAT_EQ(calEncoder(0), msg.position[2]);
    //EXPECT_FLOAT_EQ(calEncoder(1), msg.position[3]);
    EXPECT_NEAR(0, msg.position[2], 1);
    EXPECT_NEAR(0, msg.position[3], 1);
//    EXPECT_NEAR(0.27, msg.position[6],  0.1);
//    EXPECT_NEAR(0.252, msg.position[7],  0.1);

    controlMsg.calibrationMode.state = nasa_r2_common_msgs::JointControlCalibrationMode::ENABLE;
    wristJoint.setCommand(commandMsg, controlMsg);
    wristJoint.updateMeasuredState(controlMsg);
    wristJoint.updateMeasuredState(controlMsg);
    msg = wristJoint.getSimpleMeasuredState();

    EXPECT_EQ(nasa_r2_common_msgs::JointControlCalibrationMode::DISABLE, io.getLiveCoeff(calibrationModeName));

    EXPECT_NEAR(calAngle(0), msg.position[0], 0.00174532925);
    EXPECT_NEAR(calAngle(1), msg.position[1], 0.00174532925);
    //EXPECT_FLOAT_EQ(calEncoder(0), msg.position[2]);
    //EXPECT_FLOAT_EQ(calEncoder(1), msg.position[3]);
    EXPECT_NEAR(calSlider(0), msg.position[2], 1);
    EXPECT_NEAR(calSlider(1), msg.position[3], 1);
//    EXPECT_NEAR(0.27, msg.position[6],  0.1);
//    EXPECT_NEAR(0.252, msg.position[7],  0.1);

    wristJoint.updateMeasuredState(controlMsg);
    msg = wristJoint.getSimpleMeasuredState();

    EXPECT_NEAR(calAngle(0), msg.position[0], 0.005);
    EXPECT_NEAR(calAngle(1), msg.position[1], 0.005);
    //EXPECT_NEAR(calEncoder(0), msg.position[2], 1);
    //EXPECT_NEAR(calEncoder(1), msg.position[3], 1);
    EXPECT_NEAR(calSlider(0), msg.position[2], 1);
    EXPECT_NEAR(calSlider(1), msg.position[3], 1);
//    EXPECT_NEAR(calAngle(0), msg.position[6],  0.1);
//    EXPECT_NEAR(calAngle(1), msg.position[7],  0.1);

    EXPECT_FLOAT_EQ(0, msg.velocity[0]);
    EXPECT_FLOAT_EQ(0, msg.velocity[1]);
    EXPECT_FLOAT_EQ(0, msg.velocity[2]);
    EXPECT_FLOAT_EQ(0, msg.velocity[3]);
    //EXPECT_FLOAT_EQ(0, msg.velocity[4]);
    //EXPECT_FLOAT_EQ(0, msg.velocity[5]);
    //EXPECT_FLOAT_EQ(0, msg.velocity[6]);
    //EXPECT_FLOAT_EQ(0, msg.velocity[7]);
}

TEST_F(JointCommandWristTest, getCompleteMeasuredState)
{
    JointCommandWrist wristJoint(mechanism, io);

    io.setLiveCoeff("/r2/right_arm/wrist/UseHalls", 0);

    setInt16("/r2/right_arm/wrist/thumbside/IncEnc", calEncoder(0));
    setInt16("/r2/right_arm/wrist/thumbside/MotCom", 5);
    setInt16("/r2/right_arm/wrist/littleside/IncEnc", calEncoder(1));
    setInt16("/r2/right_arm/wrist/littleside/MotCom", 5);
    setUint16("/r2/right_arm/wrist/pitch/Hall", 2);
    setUint16("/r2/right_arm/wrist/yaw/Hall",   3);

    wristJoint.updateMeasuredState(controlMsg);
    sensor_msgs::JointState msg = wristJoint.getCompleteMeasuredState();

    EXPECT_EQ(12, msg.name.size());
    EXPECT_EQ(12, msg.position.size());
    EXPECT_EQ(12, msg.velocity.size());
    EXPECT_EQ(12, msg.effort.size());

    EXPECT_STREQ("/r2/right_arm/wrist/pitch",                       msg.name[0].c_str());
    EXPECT_STREQ("/r2/right_arm/wrist/yaw",                         msg.name[1].c_str());
    EXPECT_STREQ("/r2/right_arm/wrist/thumbside",                   msg.name[2].c_str());
    EXPECT_STREQ("/r2/right_arm/wrist/littleside",                  msg.name[3].c_str());
    EXPECT_STREQ("/r2/right_arm/wrist/pitch/halls",                 msg.name[4].c_str());
    EXPECT_STREQ("/r2/right_arm/wrist/yaw/halls",                   msg.name[5].c_str());
    EXPECT_STREQ("/r2/right_arm/wrist/thumbside/encoder",           msg.name[6].c_str());
    EXPECT_STREQ("/r2/right_arm/wrist/littleside/encoder",          msg.name[7].c_str());
    EXPECT_STREQ("/r2/right_arm/wrist/pitch/embeddedCommand",       msg.name[8].c_str());
    EXPECT_STREQ("/r2/right_arm/wrist/yaw/embeddedCommand",         msg.name[9].c_str());
    EXPECT_STREQ("/r2/right_arm/wrist/thumbside/embeddedCommand",   msg.name[10].c_str());
    EXPECT_STREQ("/r2/right_arm/wrist/littleside/embeddedCommand",  msg.name[11].c_str());

    EXPECT_NEAR(0, msg.position[0], 0.00174532925);
    EXPECT_NEAR(0, msg.position[1], 0.00174532925);
    EXPECT_NEAR(0, msg.position[2], 1);
    EXPECT_NEAR(0, msg.position[3], 1);
    EXPECT_NEAR(2, msg.position[4],  0.1);
    EXPECT_NEAR(3, msg.position[5],  0.1);
    EXPECT_FLOAT_EQ(calEncoder(0), msg.position[6]);
    EXPECT_FLOAT_EQ(calEncoder(1), msg.position[7]);

    controlMsg.calibrationMode.state = nasa_r2_common_msgs::JointControlCalibrationMode::ENABLE;
    wristJoint.setCommand(commandMsg, controlMsg);
    wristJoint.updateMeasuredState(controlMsg);
    wristJoint.updateMeasuredState(controlMsg);
    msg = wristJoint.getCompleteMeasuredState();

    //EXPECT_EQ(nasa_r2_common_msgs::JointControlCalibrationMode::DISABLE, io.getLiveCoeff(calibrationModeName));

    EXPECT_NEAR(calAngle(0), msg.position[0], 0.00174532925);
    EXPECT_NEAR(calAngle(1), msg.position[1], 0.00174532925);
    EXPECT_NEAR(calSlider(0), msg.position[2], 1);
    EXPECT_NEAR(calSlider(1), msg.position[3], 1);
    EXPECT_NEAR(2,  msg.position[4],  0.1);
    EXPECT_NEAR(3, msg.position[5],  0.1);
    EXPECT_FLOAT_EQ(calEncoder(0), msg.position[6]);
    EXPECT_FLOAT_EQ(calEncoder(1), msg.position[7]);

    wristJoint.updateMeasuredState(controlMsg);
    msg = wristJoint.getCompleteMeasuredState();

    EXPECT_NEAR(calAngle(0), msg.position[0], 0.005);
    EXPECT_NEAR(calAngle(1), msg.position[1], 0.005);
    EXPECT_NEAR(calSlider(0), msg.position[2], 1);
    EXPECT_NEAR(calSlider(1), msg.position[3], 1);
    EXPECT_NEAR(2, msg.position[4],  0.1);
    EXPECT_NEAR(3, msg.position[5],  0.1);
    EXPECT_NEAR(calEncoder(0), msg.position[6], 1);
    EXPECT_NEAR(calEncoder(1), msg.position[7], 1);

    EXPECT_FLOAT_EQ(0, msg.velocity[0]);
    EXPECT_FLOAT_EQ(0, msg.velocity[1]);
    EXPECT_FLOAT_EQ(0, msg.velocity[2]);
    EXPECT_FLOAT_EQ(0, msg.velocity[3]);
    EXPECT_FLOAT_EQ(0, msg.velocity[4]);
    EXPECT_FLOAT_EQ(0, msg.velocity[5]);
    EXPECT_FLOAT_EQ(0, msg.velocity[6]);
    EXPECT_FLOAT_EQ(0, msg.velocity[7]);
}

TEST_F(JointCommandWristTest, LiveCoeff)
{
    JointCommandWrist wristJoint("/r2/right_arm/wrist", io);
    io.setLiveCoeff("/r2/right_arm/wrist/UseHalls", 0);

    setInt16("/r2/right_arm/wrist/thumbside/IncEnc", calEncoder(0));
    setInt16("/r2/right_arm/wrist/thumbside/MotCom", 5);
    setInt16("/r2/right_arm/wrist/littleside/IncEnc", calEncoder(1));
    setInt16("/r2/right_arm/wrist/littleside/MotCom", 5);
    setUint16("/r2/right_arm/wrist/pitch/Hall", calAngle(0));
    setUint16("/r2/right_arm/wrist/yaw/Hall", calAngle(1));

    controlMsg.calibrationMode.state = nasa_r2_common_msgs::JointControlCalibrationMode::IGNORE;
    wristJoint.setCommand(commandMsg, controlMsg);
    wristJoint.updateMeasuredState(controlMsg);
    wristJoint.getSimpleMeasuredState();
    EXPECT_FLOAT_EQ(nasa_r2_common_msgs::JointControlCalibrationMode::IGNORE, io.getLiveCoeff(calibrationModeName));

    io.setLiveCoeff(calibrationModeName, nasa_r2_common_msgs::JointControlCalibrationMode::ENABLE);
    controlMsg.calibrationMode.state = nasa_r2_common_msgs::JointControlCalibrationMode::ENABLE;
    wristJoint.setCommand(commandMsg, controlMsg);

    EXPECT_FLOAT_EQ(nasa_r2_common_msgs::JointControlCalibrationMode::ENABLE, io.getLiveCoeff(calibrationModeName) );

    wristJoint.updateMeasuredState(controlMsg);
    wristJoint.getSimpleMeasuredState();

    EXPECT_FLOAT_EQ(nasa_r2_common_msgs::JointControlCalibrationMode::DISABLE, io.getLiveCoeff(calibrationModeName));

}

TEST_F(JointCommandWristTest, Commands)
{
    JointCommandWrist wristJoint("/r2/right_arm/wrist", io);
    io.setLiveCoeff("/r2/right_arm/wrist/UseHalls", 0);

    nasa_r2_common_msgs::JointCommand outMsg;

    commandMsg.header.stamp = ros::Time::now();

    commandMsg.name.resize(2);
    commandMsg.desiredPosition.resize(2);

    std::cout<<"testing bootloader"<<std::endl;
    controlMsg.controlMode.state = nasa_r2_common_msgs::JointControlMode::BOOTLOADER;
    controlMsg.calibrationMode.state = nasa_r2_common_msgs::JointControlCalibrationMode::ENABLE;

    commandMsg.name[0]                         = "/r2/right_arm/wrist/pitch";
    commandMsg.name[1]                         = "/r2/right_arm/wrist/yaw";

    commandMsg.desiredPosition[0]              = 0;
    commandMsg.desiredPosition[1]              = 0;

    wristJoint.setCommand(commandMsg, controlMsg);
    wristJoint.updateMeasuredState(controlMsg);
    wristJoint.getSimpleMeasuredState();

    EXPECT_FLOAT_EQ(0, getInt16("/r2/right_arm/wrist/thumbside/MotCom"));
    EXPECT_FLOAT_EQ(0, getInt16("/r2/right_arm/wrist/littleside/MotCom"));

    std::cout<<"testing park"<<std::endl;
    controlMsg.controlMode.state = nasa_r2_common_msgs::JointControlMode::PARK;

    commandMsg.name[0]                         = "/r2/right_arm/wrist/pitch";
    commandMsg.name[1]                         = "/r2/right_arm/wrist/yaw";

    commandMsg.desiredPosition[0]              = 0;
    commandMsg.desiredPosition[1]              = 0;

    wristJoint.setCommand(commandMsg, controlMsg);

    EXPECT_FLOAT_EQ(0, getInt16("/r2/right_arm/wrist/thumbside/MotCom"));
    EXPECT_FLOAT_EQ(0, getInt16("/r2/right_arm/wrist/littleside/MotCom"));

    std::cout<<"testing drive"<<std::endl;
    controlMsg.controlMode.state = nasa_r2_common_msgs::JointControlMode::DRIVE;


    std::cout<<"testing pitch/yaw commands"<<std::endl;
    commandMsg.name[0]                         = "/r2/right_arm/wrist/pitch";
    commandMsg.name[1]                         = "/r2/right_arm/wrist/yaw";

    commandMsg.desiredPosition[0]              = .1;
    commandMsg.desiredPosition[1]              = .1;

    wristJoint.setCommand(commandMsg, controlMsg);

    EXPECT_NE(0, getInt16("/r2/right_arm/wrist/thumbside/MotCom"));
    EXPECT_NE(0, getInt16("/r2/right_arm/wrist/littleside/MotCom"));

    outMsg = wristJoint.getCommandedState();

    EXPECT_EQ(2, outMsg.name.size());
    EXPECT_EQ(2, outMsg.desiredPosition.size());

    EXPECT_STREQ("/r2/right_arm/wrist/pitch", outMsg.name[0].c_str());
    EXPECT_STREQ("/r2/right_arm/wrist/yaw", outMsg.name[1].c_str());

    EXPECT_NEAR(.1, outMsg.desiredPosition[0], 0.0005);
    EXPECT_NEAR(.1, outMsg.desiredPosition[1], 0.0005);

    std::cout<<"testing actuator commands"<<std::endl;
    controlMsg.commandMode.state = nasa_r2_common_msgs::JointControlCommandMode::ACTUATOR;

    commandMsg.name[0] = "/r2/right_arm/wrist/thumbside";
    commandMsg.name[1] = "/r2/right_arm/wrist/littleside";

    commandMsg.desiredPosition[0] = calSlider(0);
    commandMsg.desiredPosition[1] = calSlider(1);

    wristJoint.setCommand(commandMsg, controlMsg);

    EXPECT_NE(0, getInt16("/r2/right_arm/wrist/thumbside/MotCom"));
    EXPECT_NE(0, getInt16("/r2/right_arm/wrist/littleside/MotCom"));

    outMsg = wristJoint.getCommandedState();

    EXPECT_EQ(2, outMsg.name.size());
    EXPECT_EQ(2, outMsg.desiredPosition.size());

    EXPECT_STREQ("/r2/right_arm/wrist/pitch", outMsg.name[0].c_str());
    EXPECT_STREQ("/r2/right_arm/wrist/yaw", outMsg.name[1].c_str());

    EXPECT_NEAR(calAngle(0), outMsg.desiredPosition[0], 0.005);
    EXPECT_NEAR(calAngle(1), outMsg.desiredPosition[1], 0.005);
}


TEST_F(JointCommandWristTest, getCapability)
{
    JointCommandWrist wristJoint("/r2/right_arm/wrist", io);

    nasa_r2_common_msgs::JointCapability msg = wristJoint.getCapability();

    EXPECT_EQ(2, msg.name.size());
    EXPECT_EQ(2, msg.positionLimitMax.size());
    EXPECT_EQ(2, msg.positionLimitMin.size());

    EXPECT_STREQ("/r2/right_arm/wrist/pitch", msg.name[0].c_str());
    EXPECT_STREQ("/r2/right_arm/wrist/yaw", msg.name[1].c_str());
    EXPECT_GE(getMotorCoeff("/r2/right_arm/wrist/UpperPitchLim"), msg.positionLimitMax[0]);
    EXPECT_FLOAT_EQ(getMotorCoeff("/r2/right_arm/wrist/LowerPitchLim"), msg.positionLimitMin[0]);
    EXPECT_FLOAT_EQ(getMotorCoeff("/r2/right_arm/wrist/UpperYawLim"), msg.positionLimitMax[1]);
    EXPECT_FLOAT_EQ(getMotorCoeff("/r2/right_arm/wrist/LowerYawLim"), msg.positionLimitMin[1]);

}

/*
TEST_F(JointCommandWristTest, setFault)
{
    // Store fully qualified names
    std::string nameIsCalibrated          = StringUtilities::makeFullyQualifiedRoboDynElement(mechanism, "IsCalibrated");
    std::string namePitchLimit            = StringUtilities::makeFullyQualifiedRoboDynElement(mechanism, "PitchLimit");
    std::string nameYawLimit              = StringUtilities::makeFullyQualifiedRoboDynElement(mechanism, "YawLimit");
    std::string nameThumbsideSliderLimit  = StringUtilities::makeFullyQualifiedRoboDynElement(mechanism, "ThumbsideSliderLimit");
    std::string nameLittlesideSliderLimit = StringUtilities::makeFullyQualifiedRoboDynElement(mechanism, "LittlesideSliderLimit");
    std::string nameSensorError           = StringUtilities::makeFullyQualifiedRoboDynElement(mechanism, "SensorError");
    std::string nameSliderDiffError       = StringUtilities::makeFullyQualifiedRoboDynElement(mechanism, "SliderDiffError");

    JointCommandWrist wristJoint(mechanism, io);

    EXPECT_TRUE(io.hasLiveCoeff(nameIsCalibrated));
    EXPECT_TRUE(io.hasLiveCoeff(namePitchLimit));
    EXPECT_TRUE(io.hasLiveCoeff(nameYawLimit));
    EXPECT_TRUE(io.hasLiveCoeff(nameLittlesideSliderLimit));
    EXPECT_TRUE(io.hasLiveCoeff(nameThumbsideSliderLimit));
    EXPECT_TRUE(io.hasLiveCoeff(nameSensorError));
    EXPECT_TRUE(io.hasLiveCoeff(nameSliderDiffError));

    EXPECT_FLOAT_EQ(0, io.getLiveCoeff(nameIsCalibrated));
    EXPECT_FLOAT_EQ(0, io.getLiveCoeff(namePitchLimit));
    EXPECT_FLOAT_EQ(0, io.getLiveCoeff(nameYawLimit));
    EXPECT_FLOAT_EQ(0, io.getLiveCoeff(nameLittlesideSliderLimit));
    EXPECT_FLOAT_EQ(0, io.getLiveCoeff(nameThumbsideSliderLimit));
    EXPECT_FLOAT_EQ(0, io.getLiveCoeff(nameSensorError));
    EXPECT_FLOAT_EQ(0, io.getLiveCoeff(nameSliderDiffError));

    setInt16("/r2/right_arm/wrist/thumbside/IncEnc", calEncoder(0));
    setInt16("/r2/right_arm/wrist/thumbside/MotCom", 5);
    setInt16("/r2/right_arm/wrist/littleside/IncEnc", calEncoder(1));
    setInt16("/r2/right_arm/wrist/littleside/MotCom", 5);
    setUint16("/r2/right_arm/wrist/pitch/Hall", calAngle(0));
    setUint16("/r2/right_arm/wrist/yaw/Hall", calAngle(1));

    //initialize to known good values

    //! @todo fix inputs so a kinematic solution can be found
    // wristJoint.updateMeasuredState();
    // wristJoint.getSimpleMeasuredState();

    EXPECT_TRUE(io.hasLiveCoeff(nameIsCalibrated));
    EXPECT_TRUE(io.hasLiveCoeff(namePitchLimit));
    EXPECT_TRUE(io.hasLiveCoeff(nameYawLimit));
    EXPECT_TRUE(io.hasLiveCoeff(nameLittlesideSliderLimit));
    EXPECT_TRUE(io.hasLiveCoeff(nameThumbsideSliderLimit));
    EXPECT_TRUE(io.hasLiveCoeff(nameSensorError));
    EXPECT_TRUE(io.hasLiveCoeff(nameSliderDiffError));

    EXPECT_FLOAT_EQ(1, io.getLiveCoeff(nameIsCalibrated));
    EXPECT_FLOAT_EQ(0, io.getLiveCoeff(namePitchLimit));
    EXPECT_FLOAT_EQ(0, io.getLiveCoeff(nameYawLimit));
    EXPECT_FLOAT_EQ(0, io.getLiveCoeff(nameLittlesideSliderLimit));
    EXPECT_FLOAT_EQ(0, io.getLiveCoeff(nameThumbsideSliderLimit));
    //EXPECT_FLOAT_EQ(0, io.getLiveCoeff(nameSensorError));
    EXPECT_FLOAT_EQ(0, io.getLiveCoeff(nameSliderDiffError));


    //set pitch above limit
    setInt16("/r2/right_arm/wrist/thumbside/IncEnc", calEncoder(0));
    setInt16("/r2/right_arm/wrist/littleside/IncEnc", calEncoder(1));


//    setInt16("/r2/right_arm/wrist/littleside/IncEnc", 11780);
//    setInt16("/r2/right_arm/wrist/littleside/MotCom", 2);
//    setInt16("/r2/right_arm/wrist/thumbside/IncEnc", 11780);
//    setInt16("/r2/right_arm/wrist/thumbside/MotCom", 4);
//    setInt16("/r2/right_arm/wrist/pitch/Hall", 0);
//    setInt16("/r2/right_arm/wrist/yaw/Hall", 0);

//    wristJoint.updateMeasuredState();
//    wristJoint.getSimpleMeasuredState();

    //test fault
}
*/

int main(int argc, char **argv){
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
