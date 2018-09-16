#include <iostream>
#include <gtest/gtest.h>
#include <boost/shared_ptr.hpp>
#include "nasa_r2_config_core/RobotInstanceFactory.h"
#include "nasa_r2_config_core/NodeRegisterManager.h"
#include "nasa_robodyn_mechanisms_core/JointControlCommandFsmGripper.h"
#include "nasa_robodyn_mechanisms_core/JointControlCommandFsmSeriesElastic.h"
#include "nasa_robodyn_mechanisms_core/JointControlCommandFsmWrist.h"
#include "nasa_robodyn_mechanisms_core/JointControlManagerFactory.h"
#include <ros/package.h>

using namespace std;

// Fake the io functions
std::map<std::string, float> testLiveCoeffs;
RobotInstancePtr             instance;

std::string getRegisterFile(std::string mechanism)
{
    return instance->configurationSafetyBasePath + RobotInstance::REGISTER_PATH + instance->mechanismRegisterMap[mechanism];
}

std::string getControlFile(std::string mechanism)
{
    return instance->configurationSafetyBasePath + RobotInstance::APIMAP_PATH + instance->mechanismControlMap[mechanism];
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

class JointControlCommandFsmTest : public ::testing::Test {
    protected:
        virtual void SetUp()
        {
            // Initialize objects
            nodeRegisterManager = boost::make_shared<NodeRegisterManager>();
            jointControlFsmMap.clear();

            // Load RobotInstance
            packagePath      = ros::package::getPath("nasa_r2_config_core");
            configPath       = packagePath + "/test/config";
            configSafetyPath = packagePath + "/test/configSafety";
            fileName         = "TestRobotInstance.xml";
            instance         = RobotInstanceFactory::createRobotInstanceFromFile(configPath, configSafetyPath, fileName);

            // Set up function pointers
            testLiveCoeffs.clear();
            io.getRegisterFile = getRegisterFile;
            io.getControlFile  = getControlFile;
            io.hasLiveCoeff    = hasLiveCoeff;
            io.getLiveCoeff    = getLiveCoeff;
            io.setLiveCoeff    = setLiveCoeff;

            // Initialize nodeRegisterManager and jointControlFsmMap
            for (unsigned int i = 0; i < instance->mechanisms.size(); i++)
            {
                mechanism = instance->mechanisms[i];
                fileName  = getRegisterFile(mechanism);
                jointType = JointControlManagerFactory::Private::getPropertyFromFile(fileName, "NodeType");

                // Simple types
                if (jointType == "SeriesElastic")
                {
                    nodeRegisterManager->addNode(mechanism, fileName);
                    jointControlFsmMap[mechanism] = boost::make_shared<JointControlCommandFsmSeriesElastic>(mechanism, io, nodeRegisterManager);
                }
                else if (jointType == "Rigid")
                {
                    nodeRegisterManager->addNode(mechanism, fileName);
                    jointControlFsmMap[mechanism] = boost::make_shared<JointControlCommandFsmSeriesElastic>(mechanism, io, nodeRegisterManager);
                }
                else if (jointType == "Gripper")
                {
                    nodeRegisterManager->addNode(mechanism, fileName);
                    jointControlFsmMap[mechanism] = boost::make_shared<JointControlCommandFsmGripper>(mechanism, io, nodeRegisterManager);
                }

                // Complex types
                else if (jointType == "Wrist")
                {
                    jointControlFsmMap[mechanism] = boost::make_shared<JointControlCommandFsmWrist>(mechanism, io);
                }
                else if (jointType == "Thumb")
                {
                    // Nothing yet
                }
                else if (jointType == "PrimaryFinger")
                {
                    // Nothing yet
                }
                else if (jointType == "SecondaryFingers")
                {
                    // Nothing yet
                }
                else
                {
                    std::stringstream err;
                    err << "Unsupported joint type [" << jointType << "] found in ControlFile";
                    RCS::Logger::log("gov.nasa.robonet.JointControlCommandFsmTest", log4cpp::Priority::ERROR, err.str());
                    ASSERT_TRUE(false);
                }
            }
        }

        virtual void TearDown()
        {
        }

        std::string            packagePath;
        std::string            configPath, configSafetyPath, fileName;
        NodeRegisterManagerPtr nodeRegisterManager;
        std::string            jointType;
        std::string            mechanism;
        std::map< std::string, boost::shared_ptr<JointControlCommandFsmInterface> > jointControlFsmMap;
        JointControlCommonInterface::IoFunctions io;
};

TEST_F(JointControlCommandFsmTest, InitialBootloader)
{
    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::BOOTLOADER, jointControlFsmMap["/r2/left_leg/joint0"]->getStates().controlMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::BOOTLOADER, jointControlFsmMap["/r2/right_leg/gripper/joint0"]->getStates().controlMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::BOOTLOADER, jointControlFsmMap["/r2/left_arm/joint0"]->getStates().controlMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::BOOTLOADER, jointControlFsmMap["/r2/neck/joint0"]->getStates().controlMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::BOOTLOADER, jointControlFsmMap["/r2/right_arm/wrist"]->getStates().controlMode.state);

    EXPECT_EQ(0, nodeRegisterManager->getControlValue("/r2/left_leg/joint0", "BootEnable"));
    EXPECT_EQ(0, nodeRegisterManager->getControlValue("/r2/left_leg/joint0", "BridgeEnable"));
    EXPECT_EQ(0, nodeRegisterManager->getControlValue("/r2/left_leg/joint0", "MotorEnable"));

    EXPECT_EQ(0, nodeRegisterManager->getControlValue("/r2/right_leg/gripper/joint0", "BootEnable"));
    EXPECT_EQ(0, nodeRegisterManager->getControlValue("/r2/right_leg/gripper/joint0", "BridgeEnable"));
    EXPECT_EQ(0, nodeRegisterManager->getControlValue("/r2/right_leg/gripper/joint0", "MotorEnable"));

    EXPECT_EQ(0, nodeRegisterManager->getControlValue("/r2/left_arm/joint0", "BootEnable"));
    EXPECT_EQ(0, nodeRegisterManager->getControlValue("/r2/left_arm/joint0", "BridgeEnable"));
    EXPECT_EQ(0, nodeRegisterManager->getControlValue("/r2/left_arm/joint0", "MotorEnable"));
    EXPECT_EQ(0, nodeRegisterManager->getControlValue("/r2/left_arm/joint0", "BrakeRelease"));

    EXPECT_EQ(0, nodeRegisterManager->getControlValue("/r2/neck/joint0", "BootEnable"));
    EXPECT_EQ(0, nodeRegisterManager->getControlValue("/r2/neck/joint0", "BridgeEnable"));
    EXPECT_EQ(0, nodeRegisterManager->getControlValue("/r2/neck/joint0", "MotorEnable"));
    EXPECT_EQ(0, nodeRegisterManager->getControlValue("/r2/neck/joint0", "BrakeRelease"));
}

TEST_F(JointControlCommandFsmTest, Off)
{
    jointControlFsmMap["/r2/left_leg/joint0"]->off();
    jointControlFsmMap["/r2/right_leg/gripper/joint0"]->off();
    jointControlFsmMap["/r2/left_arm/joint0"]->off();
    jointControlFsmMap["/r2/neck/joint0"]->off();

    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::OFF, jointControlFsmMap["/r2/left_leg/joint0"]->getStates().controlMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::OFF, jointControlFsmMap["/r2/right_leg/gripper/joint0"]->getStates().controlMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::OFF, jointControlFsmMap["/r2/left_arm/joint0"]->getStates().controlMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::OFF, jointControlFsmMap["/r2/neck/joint0"]->getStates().controlMode.state);

    EXPECT_EQ(1, nodeRegisterManager->getControlValue("/r2/left_leg/joint0", "BootEnable"));
    EXPECT_EQ(0, nodeRegisterManager->getControlValue("/r2/left_leg/joint0", "BridgeEnable"));
    EXPECT_EQ(0, nodeRegisterManager->getControlValue("/r2/left_leg/joint0", "MotorEnable"));

    EXPECT_EQ(1, nodeRegisterManager->getControlValue("/r2/right_leg/gripper/joint0", "BootEnable"));
    EXPECT_EQ(0, nodeRegisterManager->getControlValue("/r2/right_leg/gripper/joint0", "BridgeEnable"));
    EXPECT_EQ(0, nodeRegisterManager->getControlValue("/r2/right_leg/gripper/joint0", "MotorEnable"));

    EXPECT_EQ(1, nodeRegisterManager->getControlValue("/r2/left_arm/joint0", "BootEnable"));
    EXPECT_EQ(0, nodeRegisterManager->getControlValue("/r2/left_arm/joint0", "BridgeEnable"));
    EXPECT_EQ(0, nodeRegisterManager->getControlValue("/r2/left_arm/joint0", "MotorEnable"));
    EXPECT_EQ(0, nodeRegisterManager->getControlValue("/r2/left_arm/joint0", "BrakeRelease"));

    EXPECT_EQ(1, nodeRegisterManager->getControlValue("/r2/neck/joint0", "BootEnable"));
    EXPECT_EQ(0, nodeRegisterManager->getControlValue("/r2/neck/joint0", "BridgeEnable"));
    EXPECT_EQ(0, nodeRegisterManager->getControlValue("/r2/neck/joint0", "MotorEnable"));
    EXPECT_EQ(0, nodeRegisterManager->getControlValue("/r2/neck/joint0", "BrakeRelease"));
}

TEST_F(JointControlCommandFsmTest, Park)
{
    jointControlFsmMap["/r2/left_leg/joint0"]->off();
    jointControlFsmMap["/r2/right_leg/gripper/joint0"]->off();
    jointControlFsmMap["/r2/left_arm/joint0"]->off();
    jointControlFsmMap["/r2/neck/joint0"]->off();
    jointControlFsmMap["/r2/right_arm/wrist"]->off();
    jointControlFsmMap["/r2/left_leg/joint0"]->park();
    jointControlFsmMap["/r2/right_leg/gripper/joint0"]->park();
    jointControlFsmMap["/r2/left_arm/joint0"]->park();
    jointControlFsmMap["/r2/neck/joint0"]->park();
    jointControlFsmMap["/r2/right_arm/wrist"]->park();

    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::PARK, jointControlFsmMap["/r2/left_leg/joint0"]->getStates().controlMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::PARK, jointControlFsmMap["/r2/right_leg/gripper/joint0"]->getStates().controlMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::PARK, jointControlFsmMap["/r2/left_arm/joint0"]->getStates().controlMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::PARK, jointControlFsmMap["/r2/neck/joint0"]->getStates().controlMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::PARK, jointControlFsmMap["/r2/right_arm/wrist"]->getStates().controlMode.state);

    EXPECT_EQ(1, nodeRegisterManager->getControlValue("/r2/left_leg/joint0", "BootEnable"));
    EXPECT_EQ(1, nodeRegisterManager->getControlValue("/r2/left_leg/joint0", "BridgeEnable"));
    EXPECT_EQ(1, nodeRegisterManager->getControlValue("/r2/left_leg/joint0", "MotorEnable"));

    EXPECT_EQ(1, nodeRegisterManager->getControlValue("/r2/right_leg/gripper/joint0", "BootEnable"));
    EXPECT_EQ(1, nodeRegisterManager->getControlValue("/r2/right_leg/gripper/joint0", "BridgeEnable"));
    EXPECT_EQ(1, nodeRegisterManager->getControlValue("/r2/right_leg/gripper/joint0", "MotorEnable"));

    EXPECT_EQ(1, nodeRegisterManager->getControlValue("/r2/left_arm/joint0", "BootEnable"));
    EXPECT_EQ(1, nodeRegisterManager->getControlValue("/r2/left_arm/joint0", "BridgeEnable"));
    EXPECT_EQ(1, nodeRegisterManager->getControlValue("/r2/left_arm/joint0", "MotorEnable"));
    EXPECT_EQ(0, nodeRegisterManager->getControlValue("/r2/left_arm/joint0", "BrakeRelease"));

    EXPECT_EQ(1, nodeRegisterManager->getControlValue("/r2/neck/joint0", "BootEnable"));
    EXPECT_EQ(1, nodeRegisterManager->getControlValue("/r2/neck/joint0", "BridgeEnable"));
    EXPECT_EQ(1, nodeRegisterManager->getControlValue("/r2/neck/joint0", "MotorEnable"));
    EXPECT_EQ(0, nodeRegisterManager->getControlValue("/r2/neck/joint0", "BrakeRelease"));
}

TEST_F(JointControlCommandFsmTest, Neutral)
{
    jointControlFsmMap["/r2/left_leg/joint0"]->off();
    jointControlFsmMap["/r2/right_leg/gripper/joint0"]->off();
    jointControlFsmMap["/r2/left_arm/joint0"]->off();
    jointControlFsmMap["/r2/neck/joint0"]->off();
    jointControlFsmMap["/r2/left_leg/joint0"]->neutral();
    jointControlFsmMap["/r2/right_leg/gripper/joint0"]->neutral();
    jointControlFsmMap["/r2/left_arm/joint0"]->neutral();
    jointControlFsmMap["/r2/neck/joint0"]->neutral();

    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::NEUTRAL, jointControlFsmMap["/r2/left_leg/joint0"]->getStates().controlMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::NEUTRAL, jointControlFsmMap["/r2/right_leg/gripper/joint0"]->getStates().controlMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::NEUTRAL, jointControlFsmMap["/r2/left_arm/joint0"]->getStates().controlMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::NEUTRAL, jointControlFsmMap["/r2/neck/joint0"]->getStates().controlMode.state);

    EXPECT_EQ(1, nodeRegisterManager->getControlValue("/r2/left_leg/joint0", "BootEnable"));
    EXPECT_EQ(1, nodeRegisterManager->getControlValue("/r2/left_leg/joint0", "BridgeEnable"));
    EXPECT_EQ(0, nodeRegisterManager->getControlValue("/r2/left_leg/joint0", "MotorEnable"));
    EXPECT_EQ(1, nodeRegisterManager->getControlValue("/r2/left_leg/joint0", "BrakeRelease"));

    EXPECT_EQ(1, nodeRegisterManager->getControlValue("/r2/right_leg/gripper/joint0", "BootEnable"));
    EXPECT_EQ(1, nodeRegisterManager->getControlValue("/r2/right_leg/gripper/joint0", "BridgeEnable"));
    EXPECT_EQ(0, nodeRegisterManager->getControlValue("/r2/right_leg/gripper/joint0", "MotorEnable"));
    //! @todo brake should be 0, but temporary fix forces it to be 1
    EXPECT_EQ(1, nodeRegisterManager->getControlValue("/r2/right_leg/gripper/joint0", "BrakeRelease"));

    EXPECT_EQ(1, nodeRegisterManager->getControlValue("/r2/left_arm/joint0", "BootEnable"));
    EXPECT_EQ(1, nodeRegisterManager->getControlValue("/r2/left_arm/joint0", "BridgeEnable"));
    EXPECT_EQ(0, nodeRegisterManager->getControlValue("/r2/left_arm/joint0", "MotorEnable"));
    EXPECT_EQ(1, nodeRegisterManager->getControlValue("/r2/left_arm/joint0", "BrakeRelease"));

    EXPECT_EQ(1, nodeRegisterManager->getControlValue("/r2/neck/joint0", "BootEnable"));
    EXPECT_EQ(1, nodeRegisterManager->getControlValue("/r2/neck/joint0", "BridgeEnable"));
    EXPECT_EQ(0, nodeRegisterManager->getControlValue("/r2/neck/joint0", "MotorEnable"));
    EXPECT_EQ(1, nodeRegisterManager->getControlValue("/r2/neck/joint0", "BrakeRelease"));
}

TEST_F(JointControlCommandFsmTest, Drive)
{
    jointControlFsmMap["/r2/left_leg/joint0"]->off();
    jointControlFsmMap["/r2/right_leg/gripper/joint0"]->off();
    jointControlFsmMap["/r2/left_arm/joint0"]->off();
    jointControlFsmMap["/r2/neck/joint0"]->off();
    jointControlFsmMap["/r2/left_leg/joint0"]->park();
    jointControlFsmMap["/r2/right_leg/gripper/joint0"]->park();
    jointControlFsmMap["/r2/left_arm/joint0"]->park();
    jointControlFsmMap["/r2/neck/joint0"]->park();
    jointControlFsmMap["/r2/left_leg/joint0"]->drive();
    jointControlFsmMap["/r2/right_leg/gripper/joint0"]->drive();
    jointControlFsmMap["/r2/left_arm/joint0"]->drive();
    jointControlFsmMap["/r2/neck/joint0"]->drive();
    jointControlFsmMap["/r2/right_arm/wrist"]->drive();

    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::DRIVE, jointControlFsmMap["/r2/left_leg/joint0"]->getStates().controlMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::DRIVE, jointControlFsmMap["/r2/right_leg/gripper/joint0"]->getStates().controlMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::DRIVE, jointControlFsmMap["/r2/left_arm/joint0"]->getStates().controlMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::DRIVE, jointControlFsmMap["/r2/neck/joint0"]->getStates().controlMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::DRIVE, jointControlFsmMap["/r2/right_arm/wrist"]->getStates().controlMode.state);

    EXPECT_EQ(1, nodeRegisterManager->getControlValue("/r2/left_leg/joint0", "BootEnable"));
    EXPECT_EQ(1, nodeRegisterManager->getControlValue("/r2/left_leg/joint0", "BridgeEnable"));
    EXPECT_EQ(1, nodeRegisterManager->getControlValue("/r2/left_leg/joint0", "MotorEnable"));

    EXPECT_EQ(1, nodeRegisterManager->getControlValue("/r2/right_leg/gripper/joint0", "BootEnable"));
    EXPECT_EQ(1, nodeRegisterManager->getControlValue("/r2/right_leg/gripper/joint0", "BridgeEnable"));
    EXPECT_EQ(1, nodeRegisterManager->getControlValue("/r2/right_leg/gripper/joint0", "MotorEnable"));

    EXPECT_EQ(1, nodeRegisterManager->getControlValue("/r2/left_arm/joint0", "BootEnable"));
    EXPECT_EQ(1, nodeRegisterManager->getControlValue("/r2/left_arm/joint0", "BridgeEnable"));
    EXPECT_EQ(1, nodeRegisterManager->getControlValue("/r2/left_arm/joint0", "MotorEnable"));
    EXPECT_EQ(1, nodeRegisterManager->getControlValue("/r2/left_arm/joint0", "BrakeRelease"));

    EXPECT_EQ(1, nodeRegisterManager->getControlValue("/r2/neck/joint0", "BootEnable"));
    EXPECT_EQ(1, nodeRegisterManager->getControlValue("/r2/neck/joint0", "BridgeEnable"));
    EXPECT_EQ(1, nodeRegisterManager->getControlValue("/r2/neck/joint0", "MotorEnable"));
    EXPECT_EQ(1, nodeRegisterManager->getControlValue("/r2/neck/joint0", "BrakeRelease"));
}

TEST_F(JointControlCommandFsmTest, InitialMotCom)
{
    EXPECT_EQ(nasa_r2_common_msgs::JointControlCommandMode::MOTCOM, jointControlFsmMap["/r2/left_leg/joint0"]->getStates().commandMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlCommandMode::MOTCOM, jointControlFsmMap["/r2/right_leg/gripper/joint0"]->getStates().commandMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlCommandMode::MOTCOM, jointControlFsmMap["/r2/left_arm/joint0"]->getStates().commandMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlCommandMode::MOTCOM, jointControlFsmMap["/r2/neck/joint0"]->getStates().commandMode.state);

    EXPECT_EQ(0, nodeRegisterManager->getControlValue("/r2/left_leg/joint0",          "MotComSource"));
    EXPECT_EQ(0, nodeRegisterManager->getControlValue("/r2/right_leg/gripper/joint0", "MotComSource"));
    EXPECT_EQ(0, nodeRegisterManager->getControlValue("/r2/left_arm/joint0",          "MotComSource"));
    EXPECT_EQ(0, nodeRegisterManager->getControlValue("/r2/neck/joint0",              "MotComSource"));
}

TEST_F(JointControlCommandFsmTest, CantMultiLoopWithoutCoeffs)
{
    jointControlFsmMap["/r2/left_leg/joint0"]->multiLoopStep();
    jointControlFsmMap["/r2/right_leg/gripper/joint0"]->multiLoopStep();
    jointControlFsmMap["/r2/left_arm/joint0"]->multiLoopStep();
    jointControlFsmMap["/r2/neck/joint0"]->multiLoopStep();

    EXPECT_EQ(nasa_r2_common_msgs::JointControlCommandMode::MOTCOM, jointControlFsmMap["/r2/left_leg/joint0"]->getStates().commandMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlCommandMode::MOTCOM, jointControlFsmMap["/r2/right_leg/gripper/joint0"]->getStates().commandMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlCommandMode::MOTCOM, jointControlFsmMap["/r2/left_arm/joint0"]->getStates().commandMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlCommandMode::MOTCOM, jointControlFsmMap["/r2/neck/joint0"]->getStates().commandMode.state);
    EXPECT_EQ(0, nodeRegisterManager->getControlValue("/r2/left_leg/joint0",          "MotComSource"));
    EXPECT_EQ(0, nodeRegisterManager->getControlValue("/r2/right_leg/gripper/joint0", "MotComSource"));
    EXPECT_EQ(0, nodeRegisterManager->getControlValue("/r2/left_arm/joint0",          "MotComSource"));
    EXPECT_EQ(0, nodeRegisterManager->getControlValue("/r2/neck/joint0",              "MotComSource"));

    jointControlFsmMap["/r2/left_leg/joint0"]->multiLoopSmooth();
    jointControlFsmMap["/r2/right_leg/gripper/joint0"]->multiLoopSmooth();
    jointControlFsmMap["/r2/left_arm/joint0"]->multiLoopSmooth();
    jointControlFsmMap["/r2/neck/joint0"]->multiLoopSmooth();

    EXPECT_EQ(nasa_r2_common_msgs::JointControlCommandMode::MOTCOM, jointControlFsmMap["/r2/left_leg/joint0"]->getStates().commandMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlCommandMode::MOTCOM, jointControlFsmMap["/r2/right_leg/gripper/joint0"]->getStates().commandMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlCommandMode::MOTCOM, jointControlFsmMap["/r2/left_arm/joint0"]->getStates().commandMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlCommandMode::MOTCOM, jointControlFsmMap["/r2/neck/joint0"]->getStates().commandMode.state);
    EXPECT_EQ(0, nodeRegisterManager->getControlValue("/r2/left_leg/joint0",          "MotComSource"));
    EXPECT_EQ(0, nodeRegisterManager->getControlValue("/r2/right_leg/gripper/joint0", "MotComSource"));
    EXPECT_EQ(0, nodeRegisterManager->getControlValue("/r2/left_arm/joint0",          "MotComSource"));
    EXPECT_EQ(0, nodeRegisterManager->getControlValue("/r2/neck/joint0",              "MotComSource"));
}

TEST_F(JointControlCommandFsmTest, StallMode)
{
    nodeRegisterManager->nodeRegisterMap["/r2/left_leg/joint0"].status["/r2/left_leg/joint0/StatReg2"].registerValue                   = 0x0100; // 0000 0001 0000 0000
    nodeRegisterManager->nodeRegisterMap["/r2/right_leg/gripper/joint0"].status["/r2/right_leg/gripper/joint0/StatReg2"].registerValue = 0x0100; // 0000 0001 0000 0000
    nodeRegisterManager->nodeRegisterMap["/r2/left_arm/joint0"].status["/r2/left_arm/joint0/StatReg2"].registerValue                   = 0x0100; // 0000 0001 0000 0000
    nodeRegisterManager->nodeRegisterMap["/r2/neck/joint0"].status["/r2/neck/joint0/StatReg2"].registerValue                           = 0x0100; // 0000 0001 0000 0000

    jointControlFsmMap["/r2/left_leg/joint0"]->stallMode();
    jointControlFsmMap["/r2/right_leg/gripper/joint0"]->stallMode();
    jointControlFsmMap["/r2/left_arm/joint0"]->stallMode();
    jointControlFsmMap["/r2/neck/joint0"]->stallMode();

    EXPECT_EQ(nasa_r2_common_msgs::JointControlCommandMode::STALLMODE, jointControlFsmMap["/r2/left_leg/joint0"]->getStates().commandMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlCommandMode::STALLMODE, jointControlFsmMap["/r2/right_leg/gripper/joint0"]->getStates().commandMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlCommandMode::STALLMODE, jointControlFsmMap["/r2/left_arm/joint0"]->getStates().commandMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlCommandMode::STALLMODE, jointControlFsmMap["/r2/neck/joint0"]->getStates().commandMode.state);

    EXPECT_EQ(1, nodeRegisterManager->getControlValue("/r2/left_leg/joint0", "MotComSource"));
    EXPECT_EQ(1, nodeRegisterManager->getControlValue("/r2/left_leg/joint0", "ControlMode"));

    EXPECT_EQ(1, nodeRegisterManager->getControlValue("/r2/right_leg/gripper/joint0", "MotComSource"));
    EXPECT_EQ(1, nodeRegisterManager->getControlValue("/r2/right_leg/gripper/joint0", "ControlMode"));

    EXPECT_EQ(1, nodeRegisterManager->getControlValue("/r2/left_arm/joint0", "MotComSource"));
    EXPECT_EQ(1, nodeRegisterManager->getControlValue("/r2/left_arm/joint0", "ControlMode"));

    EXPECT_EQ(1, nodeRegisterManager->getControlValue("/r2/neck/joint0", "MotComSource"));
    EXPECT_EQ(1, nodeRegisterManager->getControlValue("/r2/neck/joint0", "ControlMode"));
}

TEST_F(JointControlCommandFsmTest, MultiLoopStep)
{
    nodeRegisterManager->nodeRegisterMap["/r2/left_leg/joint0"].status["/r2/left_leg/joint0/StatReg2"].registerValue                   = 0x0100; // 0000 0001 0000 0000
    nodeRegisterManager->nodeRegisterMap["/r2/right_leg/gripper/joint0"].status["/r2/right_leg/gripper/joint0/StatReg2"].registerValue = 0x0100; // 0000 0001 0000 0000
    nodeRegisterManager->nodeRegisterMap["/r2/left_arm/joint0"].status["/r2/left_arm/joint0/StatReg2"].registerValue                   = 0x0100; // 0000 0001 0000 0000
    nodeRegisterManager->nodeRegisterMap["/r2/neck/joint0"].status["/r2/neck/joint0/StatReg2"].registerValue                           = 0x0100; // 0000 0001 0000 0000

    jointControlFsmMap["/r2/left_leg/joint0"]->multiLoopStep();
    jointControlFsmMap["/r2/right_leg/gripper/joint0"]->multiLoopStep();
    jointControlFsmMap["/r2/left_arm/joint0"]->multiLoopStep();
    jointControlFsmMap["/r2/neck/joint0"]->multiLoopStep();

    EXPECT_EQ(nasa_r2_common_msgs::JointControlCommandMode::MULTILOOPSTEP, jointControlFsmMap["/r2/left_leg/joint0"]->getStates().commandMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlCommandMode::MULTILOOPSTEP, jointControlFsmMap["/r2/right_leg/gripper/joint0"]->getStates().commandMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlCommandMode::MULTILOOPSTEP, jointControlFsmMap["/r2/left_arm/joint0"]->getStates().commandMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlCommandMode::MULTILOOPSTEP, jointControlFsmMap["/r2/neck/joint0"]->getStates().commandMode.state);

    EXPECT_EQ(1, nodeRegisterManager->getControlValue("/r2/left_leg/joint0", "MotComSource"));
    EXPECT_EQ(3, nodeRegisterManager->getControlValue("/r2/left_leg/joint0", "ControlMode"));
    EXPECT_EQ(0, nodeRegisterManager->getControlValue("/r2/left_leg/joint0", "PosComVelocity"));

    EXPECT_EQ(1, nodeRegisterManager->getControlValue("/r2/right_leg/gripper/joint0", "MotComSource"));
    EXPECT_EQ(3, nodeRegisterManager->getControlValue("/r2/right_leg/gripper/joint0", "ControlMode"));

    EXPECT_EQ(1, nodeRegisterManager->getControlValue("/r2/left_arm/joint0", "MotComSource"));
    EXPECT_EQ(3, nodeRegisterManager->getControlValue("/r2/left_arm/joint0", "ControlMode"));
    EXPECT_EQ(0, nodeRegisterManager->getControlValue("/r2/left_arm/joint0", "PosComVelocity"));

    EXPECT_EQ(1, nodeRegisterManager->getControlValue("/r2/neck/joint0", "MotComSource"));
    EXPECT_EQ(3, nodeRegisterManager->getControlValue("/r2/neck/joint0", "ControlMode"));
    EXPECT_EQ(0, nodeRegisterManager->getControlValue("/r2/neck/joint0", "PosComVelocity"));
}

TEST_F(JointControlCommandFsmTest, MultiLoopSmooth)
{
    nodeRegisterManager->nodeRegisterMap["/r2/left_leg/joint0"].status["/r2/left_leg/joint0/StatReg2"].registerValue                   = 0x0100; // 0000 0001 0000 0000
    nodeRegisterManager->nodeRegisterMap["/r2/right_leg/gripper/joint0"].status["/r2/right_leg/gripper/joint0/StatReg2"].registerValue = 0x0100; // 0000 0001 0000 0000
    nodeRegisterManager->nodeRegisterMap["/r2/left_arm/joint0"].status["/r2/left_arm/joint0/StatReg2"].registerValue                   = 0x0100; // 0000 0001 0000 0000
    nodeRegisterManager->nodeRegisterMap["/r2/neck/joint0"].status["/r2/neck/joint0/StatReg2"].registerValue                           = 0x0100; // 0000 0001 0000 0000

    jointControlFsmMap["/r2/left_leg/joint0"]->multiLoopSmooth();
    jointControlFsmMap["/r2/right_leg/gripper/joint0"]->multiLoopSmooth();
    jointControlFsmMap["/r2/left_arm/joint0"]->multiLoopSmooth();
    jointControlFsmMap["/r2/neck/joint0"]->multiLoopSmooth();

    EXPECT_EQ(nasa_r2_common_msgs::JointControlCommandMode::MULTILOOPSMOOTH, jointControlFsmMap["/r2/left_leg/joint0"]->getStates().commandMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlCommandMode::MULTILOOPSTEP,   jointControlFsmMap["/r2/right_leg/gripper/joint0"]->getStates().commandMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlCommandMode::MULTILOOPSMOOTH, jointControlFsmMap["/r2/left_arm/joint0"]->getStates().commandMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlCommandMode::MULTILOOPSMOOTH, jointControlFsmMap["/r2/neck/joint0"]->getStates().commandMode.state);

    EXPECT_EQ(1, nodeRegisterManager->getControlValue("/r2/left_leg/joint0", "MotComSource"));
    EXPECT_EQ(3, nodeRegisterManager->getControlValue("/r2/left_leg/joint0", "ControlMode"));
    EXPECT_EQ(1, nodeRegisterManager->getControlValue("/r2/left_leg/joint0", "PosComVelocity"));

    EXPECT_EQ(1, nodeRegisterManager->getControlValue("/r2/right_leg/gripper/joint0", "MotComSource"));
    EXPECT_EQ(3, nodeRegisterManager->getControlValue("/r2/right_leg/gripper/joint0", "ControlMode"));

    EXPECT_EQ(1, nodeRegisterManager->getControlValue("/r2/left_arm/joint0", "MotComSource"));
    EXPECT_EQ(3, nodeRegisterManager->getControlValue("/r2/left_arm/joint0", "ControlMode"));
    EXPECT_EQ(1, nodeRegisterManager->getControlValue("/r2/left_arm/joint0", "PosComVelocity"));

    EXPECT_EQ(1, nodeRegisterManager->getControlValue("/r2/neck/joint0", "MotComSource"));
    EXPECT_EQ(3, nodeRegisterManager->getControlValue("/r2/neck/joint0", "ControlMode"));
    EXPECT_EQ(1, nodeRegisterManager->getControlValue("/r2/neck/joint0", "PosComVelocity"));
}

TEST_F(JointControlCommandFsmTest, InitToDisableCalibration)
{
    EXPECT_EQ(nasa_r2_common_msgs::JointControlCalibrationMode::DISABLE, jointControlFsmMap["/r2/left_leg/joint0"]->getStates().calibrationMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlCalibrationMode::DISABLE, jointControlFsmMap["/r2/right_leg/gripper/joint0"]->getStates().calibrationMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlCalibrationMode::DISABLE, jointControlFsmMap["/r2/left_arm/joint0"]->getStates().calibrationMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlCalibrationMode::DISABLE, jointControlFsmMap["/r2/neck/joint0"]->getStates().calibrationMode.state);

    EXPECT_EQ(0, nodeRegisterManager->getControlValue("/r2/left_leg/joint0",          "CalibrationMode"));
    EXPECT_EQ(0, nodeRegisterManager->getControlValue("/r2/right_leg/gripper/joint0", "CalibrationMode"));
    EXPECT_EQ(0, nodeRegisterManager->getControlValue("/r2/left_arm/joint0",          "CalibrationMode"));
    EXPECT_EQ(0, nodeRegisterManager->getControlValue("/r2/neck/joint0",              "CalibrationMode"));
}

TEST_F(JointControlCommandFsmTest, EnableCalibrationModeAndBack)
{
    jointControlFsmMap["/r2/left_leg/joint0"]->enableCalibrationMode();
    jointControlFsmMap["/r2/right_leg/gripper/joint0"]->enableCalibrationMode();
    jointControlFsmMap["/r2/left_arm/joint0"]->enableCalibrationMode();
    jointControlFsmMap["/r2/neck/joint0"]->enableCalibrationMode();

    EXPECT_EQ(nasa_r2_common_msgs::JointControlCalibrationMode::ENABLE, jointControlFsmMap["/r2/left_leg/joint0"]->getStates().calibrationMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlCalibrationMode::ENABLE, jointControlFsmMap["/r2/right_leg/gripper/joint0"]->getStates().calibrationMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlCalibrationMode::ENABLE, jointControlFsmMap["/r2/left_arm/joint0"]->getStates().calibrationMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlCalibrationMode::ENABLE, jointControlFsmMap["/r2/neck/joint0"]->getStates().calibrationMode.state);

    EXPECT_EQ(1, nodeRegisterManager->getControlValue("/r2/left_leg/joint0",          "CalibrationMode"));
    EXPECT_EQ(1, nodeRegisterManager->getControlValue("/r2/right_leg/gripper/joint0", "CalibrationMode"));
    EXPECT_EQ(1, nodeRegisterManager->getControlValue("/r2/left_arm/joint0",          "CalibrationMode"));
    EXPECT_EQ(1, nodeRegisterManager->getControlValue("/r2/neck/joint0",              "CalibrationMode"));

    jointControlFsmMap["/r2/left_leg/joint0"]->disableCalibrationMode();
    jointControlFsmMap["/r2/right_leg/gripper/joint0"]->disableCalibrationMode();
    jointControlFsmMap["/r2/left_arm/joint0"]->disableCalibrationMode();
    jointControlFsmMap["/r2/neck/joint0"]->disableCalibrationMode();

    EXPECT_EQ(nasa_r2_common_msgs::JointControlCalibrationMode::DISABLE, jointControlFsmMap["/r2/left_leg/joint0"]->getStates().calibrationMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlCalibrationMode::DISABLE, jointControlFsmMap["/r2/right_leg/gripper/joint0"]->getStates().calibrationMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlCalibrationMode::DISABLE, jointControlFsmMap["/r2/left_arm/joint0"]->getStates().calibrationMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlCalibrationMode::DISABLE, jointControlFsmMap["/r2/neck/joint0"]->getStates().calibrationMode.state);

    EXPECT_EQ(0, nodeRegisterManager->getControlValue("/r2/left_leg/joint0",          "CalibrationMode"));
    EXPECT_EQ(0, nodeRegisterManager->getControlValue("/r2/right_leg/gripper/joint0", "CalibrationMode"));
    EXPECT_EQ(0, nodeRegisterManager->getControlValue("/r2/left_arm/joint0",          "CalibrationMode"));
    EXPECT_EQ(0, nodeRegisterManager->getControlValue("/r2/neck/joint0",              "CalibrationMode"));
}

TEST_F(JointControlCommandFsmTest, InitToDisableClearFault)
{
    EXPECT_EQ(nasa_r2_common_msgs::JointControlClearFaultMode::DISABLE, jointControlFsmMap["/r2/left_leg/joint0"]->getStates().clearFaultMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlClearFaultMode::DISABLE, jointControlFsmMap["/r2/right_leg/gripper/joint0"]->getStates().clearFaultMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlClearFaultMode::DISABLE, jointControlFsmMap["/r2/left_arm/joint0"]->getStates().clearFaultMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlClearFaultMode::DISABLE, jointControlFsmMap["/r2/neck/joint0"]->getStates().clearFaultMode.state);

    EXPECT_EQ(0, nodeRegisterManager->getControlValue("/r2/left_leg/joint0",          "ClearFault"));
    EXPECT_EQ(0, nodeRegisterManager->getControlValue("/r2/right_leg/gripper/joint0", "ClearFault"));
    EXPECT_EQ(0, nodeRegisterManager->getControlValue("/r2/left_arm/joint0",          "ClearFault"));
    EXPECT_EQ(0, nodeRegisterManager->getControlValue("/r2/neck/joint0",              "ClearFault"));
}

TEST_F(JointControlCommandFsmTest, EnableClearFaultModeAndBack)
{
    jointControlFsmMap["/r2/left_leg/joint0"]->enableClearFaultMode();
    jointControlFsmMap["/r2/right_leg/gripper/joint0"]->enableClearFaultMode();
    jointControlFsmMap["/r2/left_arm/joint0"]->enableClearFaultMode();
    jointControlFsmMap["/r2/neck/joint0"]->enableClearFaultMode();

    EXPECT_EQ(nasa_r2_common_msgs::JointControlClearFaultMode::ENABLE, jointControlFsmMap["/r2/left_leg/joint0"]->getStates().clearFaultMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlClearFaultMode::ENABLE, jointControlFsmMap["/r2/right_leg/gripper/joint0"]->getStates().clearFaultMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlClearFaultMode::ENABLE, jointControlFsmMap["/r2/left_arm/joint0"]->getStates().clearFaultMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlClearFaultMode::ENABLE, jointControlFsmMap["/r2/neck/joint0"]->getStates().clearFaultMode.state);

    EXPECT_EQ(1, nodeRegisterManager->getControlValue("/r2/left_leg/joint0",          "ClearFault"));
    EXPECT_EQ(1, nodeRegisterManager->getControlValue("/r2/right_leg/gripper/joint0", "ClearFault"));
    EXPECT_EQ(1, nodeRegisterManager->getControlValue("/r2/left_arm/joint0",          "ClearFault"));
    EXPECT_EQ(1, nodeRegisterManager->getControlValue("/r2/neck/joint0",              "ClearFault"));

    jointControlFsmMap["/r2/left_leg/joint0"]->disableClearFaultMode();
    jointControlFsmMap["/r2/right_leg/gripper/joint0"]->disableClearFaultMode();
    jointControlFsmMap["/r2/left_arm/joint0"]->disableClearFaultMode();
    jointControlFsmMap["/r2/neck/joint0"]->disableClearFaultMode();

    EXPECT_EQ(nasa_r2_common_msgs::JointControlClearFaultMode::DISABLE, jointControlFsmMap["/r2/left_leg/joint0"]->getStates().clearFaultMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlClearFaultMode::DISABLE, jointControlFsmMap["/r2/right_leg/gripper/joint0"]->getStates().clearFaultMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlClearFaultMode::DISABLE, jointControlFsmMap["/r2/left_arm/joint0"]->getStates().clearFaultMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlClearFaultMode::DISABLE, jointControlFsmMap["/r2/neck/joint0"]->getStates().clearFaultMode.state);

    EXPECT_EQ(0, nodeRegisterManager->getControlValue("/r2/left_leg/joint0",          "ClearFault"));
    EXPECT_EQ(0, nodeRegisterManager->getControlValue("/r2/right_leg/gripper/joint0", "ClearFault"));
    EXPECT_EQ(0, nodeRegisterManager->getControlValue("/r2/left_arm/joint0",          "ClearFault"));
    EXPECT_EQ(0, nodeRegisterManager->getControlValue("/r2/neck/joint0",              "ClearFault"));
}

int main(int argc, char **argv){
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
