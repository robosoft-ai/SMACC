#include <gtest/gtest.h>
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include "nasa_r2_config_core/RobotInstanceFactory.h"
#include "nasa_r2_config_core/CoeffMapLoader.h"
#include "nasa_robodyn_mechanisms_core/JointCommandFinger.h"
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
//    std::cout << "getInt16 (" << item << ")" << std::endl;
    if (testInt16.find(item) != testInt16.end())
        return testInt16[item];
    else
        return 0.f;
}

float getUint16(std::string item)
{
    if (testUint16.find(item) != testUint16.end())
        return testUint16[item];
    else
        return 0.f;
}

void setInt16(std::string item, int16_t value)
{
//    std::cout << "setInt16 (" << item << "): " << value << std::endl;
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
    return testLiveCoeffs.find(coeff) != testLiveCoeffs.end();
}

float getLiveCoeff(std::string coeff)
{
    if (hasLiveCoeff(coeff))
    {
//        std::cout << "GetLiveCoeff (" << coeff << "): " << testLiveCoeffs[coeff] << std::endl;
        return testLiveCoeffs[coeff];
    }
    else
    {
        throw std::runtime_error(std::string("GetLiveCoeff: coeff (") + coeff + ") doesn't exist");
        return -12345.f;
    }
}

void setLiveCoeff(std::string coeff, float value)
{
//    std::cout << "SetLiveCoeff (" << coeff << "): " << value << std::endl;
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

void tubeTare(std::string mechanism, JointCommandInterface::IoFunctions& io)
{
    std::vector<std::string> roboDynActuators = io.getActuatorNames(mechanism);
    for (unsigned int i = 0; i < roboDynActuators.size(); ++i)
    {
        setLiveCoeff(StringUtilities::makeFullyQualifiedRoboDynElement(roboDynActuators[i], "EncoderOffset"), 0.);
        setLiveCoeff(StringUtilities::makeFullyQualifiedRoboDynElement(roboDynActuators[i], "SliderOffset"), 0.);
        setLiveCoeff(StringUtilities::makeFullyQualifiedRoboDynElement(roboDynActuators[i], "SliderTarePosition"), 0.);
        setLiveCoeff(StringUtilities::makeFullyQualifiedRoboDynElement(roboDynActuators[i], "TensionOffset"), 0.);
    }
    setLiveCoeff(StringUtilities::makeFullyQualifiedRoboDynElement(mechanism, "IsCalibrated"), 1);
}

class JointCommandFingerTest : public ::testing::Test {
    protected:
        virtual void SetUp()
        {
            packagePath      = ros::package::getPath("nasa_r2_config_core");
            configPath       = packagePath + "/test/config";
            configSafetyPath = packagePath + "/test/configSafety";
            fileName         = "TestRobotInstanceFinger.xml";
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
            mechanism = "/r2/right_arm/hand/index";

            // default control msg
            controlMsg.calibrationMode.state = nasa_r2_common_msgs::JointControlCalibrationMode::DISABLE;
            controlMsg.clearFaultMode.state = nasa_r2_common_msgs::JointControlClearFaultMode::DISABLE;
            controlMsg.coeffState.state = nasa_r2_common_msgs::JointControlCoeffState::LOADED;
            controlMsg.commandMode.state = nasa_r2_common_msgs::JointControlCommandMode::MULTILOOPSMOOTH;
            controlMsg.controlMode.state = nasa_r2_common_msgs::JointControlMode::DRIVE;

            // default command msg
            commandMsg.name.push_back("/r2/right_arm/hand/index/yaw");
            commandMsg.name.push_back("/r2/right_arm/hand/index/proximal");
            commandMsg.name.push_back("/r2/right_arm/hand/index/medial");
            commandMsg.derivativeGain.resize(3, 0.);
            commandMsg.desiredPosition.resize(3, 0.);
            commandMsg.desiredPositionVelocityLimit.resize(3, 0.);
            commandMsg.feedForwardTorque.resize(3, 0.);
            commandMsg.integralGain.resize(3, 0.);
            commandMsg.positionLoopTorqueLimit.resize(3, 0.);
            commandMsg.positionLoopWindupLimit.resize(3, 0.);
            commandMsg.proportionalGain.resize(3, 0.);
            commandMsg.torqueLoopVelocityLimit.resize(3, 0.);
        }

        virtual void TearDown()
        {
            testInt16.clear();
            testUint16.clear();
            testLiveCoeffs.clear();
            motorCoeffMap.reset();
            brainstemCoeffMap.reset();
            instance.reset();
        }

        string packagePath, configPath, configSafetyPath, fileName;
        string mechanism;
        JointCommandInterface::IoFunctions io;
        nasa_r2_common_msgs::JointControlData controlMsg;
        nasa_r2_common_msgs::JointCommand commandMsg;

        typedef JointCommandFinger<3> index_type;
};

TEST_F(JointCommandFingerTest, Constructor)
{
    ASSERT_NO_THROW(index_type index(mechanism, io));
    //! Missing IO function definitions
    io.getInt16 = 0;
    EXPECT_THROW(index_type index(mechanism, io), std::invalid_argument);
}

TEST_F(JointCommandFingerTest, updateMeasuredState)
{
    index_type index(mechanism, io);
    EXPECT_NO_THROW(index.updateMeasuredState(controlMsg));

    setLiveCoeff(StringUtilities::makeFullyQualifiedRoboDynElement(mechanism, "IsCalibrated"), 1);
    EXPECT_THROW(index.updateMeasuredState(controlMsg), std::runtime_error);

    tubeTare(mechanism, io);
    ASSERT_NO_THROW(index.updateMeasuredState(controlMsg));
}

TEST_F(JointCommandFingerTest, getSimpleMeasuredState)
{
    index_type index(mechanism, io);
    tubeTare(mechanism, io);
    index.updateMeasuredState(controlMsg);

    sensor_msgs::JointState js = index.getSimpleMeasuredState();
    EXPECT_EQ(7, js.name.size());
    EXPECT_EQ(7, js.effort.size());
    EXPECT_EQ(7, js.position.size());
    EXPECT_EQ(7, js.velocity.size());

    // don't need to check the values because it is checked in another test
}

TEST_F(JointCommandFingerTest, getCompleteMeasuredState)
{
    index_type index(mechanism, io);
    tubeTare(mechanism, io);
    index.updateMeasuredState(controlMsg);

    sensor_msgs::JointState js = index.getCompleteMeasuredState();
    EXPECT_EQ(14, js.name.size());
    EXPECT_EQ(14, js.effort.size());
    EXPECT_EQ(14, js.position.size());
    EXPECT_EQ(14, js.velocity.size());
}

TEST_F(JointCommandFingerTest, setCommand)
{
    index_type index(mechanism, io);
    tubeTare(mechanism, io);
    index.updateMeasuredState(controlMsg);

    index.setCommand(commandMsg, controlMsg);
    nasa_r2_common_msgs::JointCommand cm = index.getCommandedState();
    EXPECT_EQ(commandMsg.derivativeGain, cm.derivativeGain);
    EXPECT_EQ(commandMsg.desiredPosition, cm.desiredPosition);
    EXPECT_EQ(commandMsg.desiredPositionVelocityLimit, cm.desiredPositionVelocityLimit);
    EXPECT_EQ(commandMsg.feedForwardTorque, cm.feedForwardTorque);
    EXPECT_EQ(commandMsg.integralGain, cm.integralGain);
    EXPECT_EQ(commandMsg.name, cm.name);
    EXPECT_EQ(commandMsg.positionLoopTorqueLimit, cm.positionLoopTorqueLimit);
    EXPECT_EQ(commandMsg.positionLoopWindupLimit, cm.positionLoopWindupLimit);
    EXPECT_EQ(commandMsg.proportionalGain, cm.proportionalGain);
    EXPECT_EQ(commandMsg.torqueLoopVelocityLimit, cm.torqueLoopVelocityLimit);
}

TEST_F(JointCommandFingerTest, getCapability)
{
    index_type index(mechanism, io);

    nasa_r2_common_msgs::JointCapability msg = index.getCapability();

    EXPECT_EQ(3, msg.name.size());
    EXPECT_EQ(3, msg.positionLimitMax.size());
    EXPECT_EQ(3, msg.positionLimitMin.size());

    EXPECT_STREQ("/r2/right_arm/hand/index/yaw", msg.name[0].c_str());
    EXPECT_STREQ("/r2/right_arm/hand/index/proximal", msg.name[1].c_str());
    EXPECT_STREQ("/r2/right_arm/hand/index/medial", msg.name[2].c_str());
}


TEST_F(JointCommandFingerTest, setFaultState)
{
    // doesn't do anything
}

int main(int argc, char **argv){
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
