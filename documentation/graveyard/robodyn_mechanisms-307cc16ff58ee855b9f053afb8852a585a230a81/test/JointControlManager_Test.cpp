#include <iostream>
#include <gtest/gtest.h>
#include <boost/shared_ptr.hpp>
#include "nasa_r2_config_core/RobotInstanceFactory.h"
#include "nasa_r2_config_core/NodeRegisterManager.h"
#include "nasa_robodyn_mechanisms_core/JointControlManagerGripper.h"
#include "nasa_robodyn_mechanisms_core/JointControlManagerSeriesElastic.h"
#include "nasa_robodyn_mechanisms_core/JointControlManagerWrist.h"
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

class JointControlManagerTest : public ::testing::Test {
    protected:
        virtual void SetUp()
        {
            // Initialize objects
            nodeRegisterManager = boost::make_shared<NodeRegisterManager>();
            jointControlManagerMap.clear();
            timeLimit = 0.05;
            ros::Time::init();

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

            // Initialize nodeRegisterManager and jointControlManagerMap
            for (unsigned int i = 0; i < instance->mechanisms.size(); i++)
            {
                mechanism = instance->mechanisms[i];
                ASSERT_NO_THROW(jointControlManagerMap[mechanism] = JointControlManagerFactory::generate(mechanism, io, timeLimit, nodeRegisterManager));

                // Fake no FPGA faults
                nodeRegisterManager->nodeRegisterMap[mechanism].status[mechanism+"/StatReg1"].registerValue = 0x9000; // 1001 0000 0000 0000

                if(mechanism == "/r2/right_arm/wrist")
                {
                    // Fake live coeffs
                    io.setLiveCoeff(StringUtilities::makeFullyQualifiedRoboDynElement(mechanism, "CalibrationState"),      nasa_r2_common_msgs::JointControlCalibrationMode::IGNORE);
                    io.setLiveCoeff(StringUtilities::makeFullyQualifiedRoboDynElement(mechanism, "PitchLimit"),            0);
                    io.setLiveCoeff(StringUtilities::makeFullyQualifiedRoboDynElement(mechanism, "YawLimit"),              0);
                    io.setLiveCoeff(StringUtilities::makeFullyQualifiedRoboDynElement(mechanism, "LittlesideSliderLimit"), 0);
                    io.setLiveCoeff(StringUtilities::makeFullyQualifiedRoboDynElement(mechanism, "ThumbsideSliderLimit"),  0);
                    io.setLiveCoeff(StringUtilities::makeFullyQualifiedRoboDynElement(mechanism, "SensorError"),           0);
                    io.setLiveCoeff(StringUtilities::makeFullyQualifiedRoboDynElement(mechanism, "SliderDiffError"),       0);
                }
            }
        }

        virtual void TearDown()
        {
        }

        std::string                              packagePath;
        std::string                              configPath, configSafetyPath, fileName;
        NodeRegisterManagerPtr                   nodeRegisterManager;
        std::string                              mechanism;
        JointControlManagerMap                   jointControlManagerMap;
        nasa_r2_common_msgs::JointControlData    commandMsg;
        double                                   timeLimit;
        JointControlCommonInterface::IoFunctions io;
};

TEST_F(JointControlManagerTest, CheckConstantValues)
{
    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::BOOTLOADER, jointControlManagerMap["/r2/left_leg/joint0"]->getActualStates().controlMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::BOOTLOADER, jointControlManagerMap["/r2/right_leg/gripper/joint0"]->getActualStates().controlMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::BOOTLOADER, jointControlManagerMap["/r2/left_arm/joint0"]->getActualStates().controlMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::BOOTLOADER, jointControlManagerMap["/r2/neck/joint0"]->getActualStates().controlMode.state);

    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::BOOTLOADER, jointControlManagerMap["/r2/left_leg/joint0"]->getCommandStates().controlMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::BOOTLOADER, jointControlManagerMap["/r2/right_leg/gripper/joint0"]->getCommandStates().controlMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::BOOTLOADER, jointControlManagerMap["/r2/left_arm/joint0"]->getCommandStates().controlMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::BOOTLOADER, jointControlManagerMap["/r2/neck/joint0"]->getCommandStates().controlMode.state);

    EXPECT_EQ(0, nodeRegisterManager->getControlValue("/r2/left_leg/joint0", "InitAPS2toAPS1"));

    EXPECT_EQ(1, nodeRegisterManager->getControlValue("/r2/right_leg/gripper/joint0", "CrabEnable"));
    EXPECT_EQ(1, nodeRegisterManager->getControlValue("/r2/right_leg/gripper/joint0", "AtiEnable"));

    EXPECT_EQ(0, nodeRegisterManager->getControlValue("/r2/left_arm/joint0", "InitAPS2toAPS1"));

    EXPECT_EQ(0, nodeRegisterManager->getControlValue("/r2/neck/joint0", "InitAPS2toAPS1"));
}

TEST_F(JointControlManagerTest, InitialControlModeStates)
{
    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::BOOTLOADER, jointControlManagerMap["/r2/left_leg/joint0"]->getActualStates().controlMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::BOOTLOADER, jointControlManagerMap["/r2/right_leg/gripper/joint0"]->getActualStates().controlMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::BOOTLOADER, jointControlManagerMap["/r2/left_arm/joint0"]->getActualStates().controlMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::BOOTLOADER, jointControlManagerMap["/r2/neck/joint0"]->getActualStates().controlMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::BOOTLOADER, jointControlManagerMap["/r2/right_arm/wrist"]->getActualStates().controlMode.state);

    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::BOOTLOADER, jointControlManagerMap["/r2/left_leg/joint0"]->getCommandStates().controlMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::BOOTLOADER, jointControlManagerMap["/r2/right_leg/gripper/joint0"]->getCommandStates().controlMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::BOOTLOADER, jointControlManagerMap["/r2/left_arm/joint0"]->getCommandStates().controlMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::BOOTLOADER, jointControlManagerMap["/r2/neck/joint0"]->getCommandStates().controlMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::BOOTLOADER, jointControlManagerMap["/r2/right_arm/wrist"]->getCommandStates().controlMode.state);

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

TEST_F(JointControlManagerTest, CheckFpgaFaults)
{
    std::cout << "/r2/left_leg/joint0:          " << jointControlManagerMap["/r2/left_leg/joint0"]->buildFaultString() << std::endl;
    std::cout << "/r2/right_leg/gripper/joint0: " << jointControlManagerMap["/r2/right_leg/gripper/joint0"]->buildFaultString() << std::endl;
    std::cout << "/r2/left_arm/joint0:          " << jointControlManagerMap["/r2/left_arm/joint0"]->buildFaultString() << std::endl;
    std::cout << "/r2/neck/joint0:              " << jointControlManagerMap["/r2/neck/joint0"]->buildFaultString() << std::endl;

    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::BOOTLOADER, jointControlManagerMap["/r2/left_leg/joint0"]->getActualStates().controlMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::BOOTLOADER, jointControlManagerMap["/r2/right_leg/gripper/joint0"]->getActualStates().controlMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::BOOTLOADER, jointControlManagerMap["/r2/left_arm/joint0"]->getActualStates().controlMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::BOOTLOADER, jointControlManagerMap["/r2/neck/joint0"]->getActualStates().controlMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::BOOTLOADER, jointControlManagerMap["/r2/left_leg/joint0"]->getCommandStates().controlMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::BOOTLOADER, jointControlManagerMap["/r2/right_leg/gripper/joint0"]->getCommandStates().controlMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::BOOTLOADER, jointControlManagerMap["/r2/left_arm/joint0"]->getCommandStates().controlMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::BOOTLOADER, jointControlManagerMap["/r2/neck/joint0"]->getCommandStates().controlMode.state);

    nodeRegisterManager->nodeRegisterMap["/r2/left_leg/joint0"].status["/r2/left_leg/joint0/StatReg1"].registerValue                   = 0x0000; // 0000 0000 0000 0000
    nodeRegisterManager->nodeRegisterMap["/r2/right_leg/gripper/joint0"].status["/r2/right_leg/gripper/joint0/StatReg1"].registerValue = 0x0000; // 0000 0000 0000 0000
    nodeRegisterManager->nodeRegisterMap["/r2/left_arm/joint0"].status["/r2/left_arm/joint0/StatReg1"].registerValue                   = 0x0000; // 0000 0000 0000 0000
    nodeRegisterManager->nodeRegisterMap["/r2/neck/joint0"].status["/r2/neck/joint0/StatReg1"].registerValue                           = 0x0000; // 0000 0000 0000 0000

    jointControlManagerMap["/r2/left_leg/joint0"]->verifyStates();
    jointControlManagerMap["/r2/right_leg/gripper/joint0"]->verifyStates();
    jointControlManagerMap["/r2/left_arm/joint0"]->verifyStates();
    jointControlManagerMap["/r2/neck/joint0"]->verifyStates();

    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::FAULTED,    jointControlManagerMap["/r2/left_leg/joint0"]->getActualStates().controlMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::FAULTED,    jointControlManagerMap["/r2/right_leg/gripper/joint0"]->getActualStates().controlMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::FAULTED,    jointControlManagerMap["/r2/left_arm/joint0"]->getActualStates().controlMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::FAULTED,    jointControlManagerMap["/r2/neck/joint0"]->getActualStates().controlMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::BOOTLOADER, jointControlManagerMap["/r2/left_leg/joint0"]->getCommandStates().controlMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::BOOTLOADER, jointControlManagerMap["/r2/right_leg/gripper/joint0"]->getCommandStates().controlMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::BOOTLOADER, jointControlManagerMap["/r2/left_arm/joint0"]->getCommandStates().controlMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::BOOTLOADER, jointControlManagerMap["/r2/neck/joint0"]->getCommandStates().controlMode.state);

    std::cout << "/r2/left_leg/joint0:          " << jointControlManagerMap["/r2/left_leg/joint0"]->buildFaultString() << std::endl;
    std::cout << "/r2/right_leg/gripper/joint0: " << jointControlManagerMap["/r2/right_leg/gripper/joint0"]->buildFaultString() << std::endl;
    std::cout << "/r2/left_arm/joint0:          " << jointControlManagerMap["/r2/left_arm/joint0"]->buildFaultString() << std::endl;
    std::cout << "/r2/neck/joint0:              " << jointControlManagerMap["/r2/neck/joint0"]->buildFaultString() << std::endl;
}

TEST_F(JointControlManagerTest, Off)
{
    commandMsg.controlMode.state = nasa_r2_common_msgs::JointControlMode::OFF;
    jointControlManagerMap["/r2/left_leg/joint0"]->setCommandStates(commandMsg);
    jointControlManagerMap["/r2/right_leg/gripper/joint0"]->setCommandStates(commandMsg);
    jointControlManagerMap["/r2/left_arm/joint0"]->setCommandStates(commandMsg);
    jointControlManagerMap["/r2/neck/joint0"]->setCommandStates(commandMsg);
    jointControlManagerMap["/r2/right_arm/wrist"]->setCommandStates(commandMsg);

    io.setLiveCoeff(StringUtilities::makeFullyQualifiedRoboDynElement("/r2/right_arm/wrist", "CalibrationState"), nasa_r2_common_msgs::JointControlCalibrationMode::DISABLE);

    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::OFF, jointControlManagerMap["/r2/left_leg/joint0"]->getActualStates().controlMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::OFF, jointControlManagerMap["/r2/right_leg/gripper/joint0"]->getActualStates().controlMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::OFF, jointControlManagerMap["/r2/left_arm/joint0"]->getActualStates().controlMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::OFF, jointControlManagerMap["/r2/neck/joint0"]->getActualStates().controlMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::OFF, jointControlManagerMap["/r2/right_arm/wrist"]->getActualStates().controlMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::OFF, jointControlManagerMap["/r2/left_leg/joint0"]->getCommandStates().controlMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::OFF, jointControlManagerMap["/r2/right_leg/gripper/joint0"]->getCommandStates().controlMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::OFF, jointControlManagerMap["/r2/left_arm/joint0"]->getCommandStates().controlMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::OFF, jointControlManagerMap["/r2/neck/joint0"]->getCommandStates().controlMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::OFF, jointControlManagerMap["/r2/right_arm/wrist"]->getCommandStates().controlMode.state);

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

    // Back into BOOTLOADER (changed with #RDEV-1239)
    commandMsg.controlMode.state = nasa_r2_common_msgs::JointControlMode::BOOTLOADER;
    jointControlManagerMap["/r2/left_leg/joint0"]->setCommandStates(commandMsg);
    jointControlManagerMap["/r2/right_leg/gripper/joint0"]->setCommandStates(commandMsg);
    jointControlManagerMap["/r2/left_arm/joint0"]->setCommandStates(commandMsg);
    jointControlManagerMap["/r2/neck/joint0"]->setCommandStates(commandMsg);
    jointControlManagerMap["/r2/right_arm/wrist"]->setCommandStates(commandMsg);

    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::BOOTLOADER, jointControlManagerMap["/r2/left_leg/joint0"]->getActualStates().controlMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::BOOTLOADER, jointControlManagerMap["/r2/right_leg/gripper/joint0"]->getActualStates().controlMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::BOOTLOADER, jointControlManagerMap["/r2/left_arm/joint0"]->getActualStates().controlMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::BOOTLOADER, jointControlManagerMap["/r2/neck/joint0"]->getActualStates().controlMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::PARK, jointControlManagerMap["/r2/right_arm/wrist"]->getActualStates().controlMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::BOOTLOADER, jointControlManagerMap["/r2/left_leg/joint0"]->getCommandStates().controlMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::BOOTLOADER, jointControlManagerMap["/r2/right_leg/gripper/joint0"]->getCommandStates().controlMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::BOOTLOADER, jointControlManagerMap["/r2/left_arm/joint0"]->getCommandStates().controlMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::BOOTLOADER, jointControlManagerMap["/r2/neck/joint0"]->getCommandStates().controlMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::PARK, jointControlManagerMap["/r2/right_arm/wrist"]->getCommandStates().controlMode.state);
}

TEST_F(JointControlManagerTest, Park)
{
    // Can't PARK from BOOTLOADER (SEA)
    commandMsg.controlMode.state = nasa_r2_common_msgs::JointControlMode::PARK;
    jointControlManagerMap["/r2/left_leg/joint0"]->setCommandStates(commandMsg);
    jointControlManagerMap["/r2/right_leg/gripper/joint0"]->setCommandStates(commandMsg);
    jointControlManagerMap["/r2/left_arm/joint0"]->setCommandStates(commandMsg);
    jointControlManagerMap["/r2/neck/joint0"]->setCommandStates(commandMsg);
    jointControlManagerMap["/r2/right_arm/wrist"]->setCommandStates(commandMsg);

    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::BOOTLOADER, jointControlManagerMap["/r2/left_leg/joint0"]->getActualStates().controlMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::BOOTLOADER, jointControlManagerMap["/r2/right_leg/gripper/joint0"]->getActualStates().controlMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::BOOTLOADER, jointControlManagerMap["/r2/left_arm/joint0"]->getActualStates().controlMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::BOOTLOADER, jointControlManagerMap["/r2/neck/joint0"]->getActualStates().controlMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::PARK,       jointControlManagerMap["/r2/right_arm/wrist"]->getActualStates().controlMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::BOOTLOADER, jointControlManagerMap["/r2/left_leg/joint0"]->getCommandStates().controlMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::BOOTLOADER, jointControlManagerMap["/r2/right_leg/gripper/joint0"]->getCommandStates().controlMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::BOOTLOADER, jointControlManagerMap["/r2/left_arm/joint0"]->getCommandStates().controlMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::BOOTLOADER, jointControlManagerMap["/r2/neck/joint0"]->getCommandStates().controlMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::PARK,       jointControlManagerMap["/r2/right_arm/wrist"]->getCommandStates().controlMode.state);

    // Can PARK through OFF
    commandMsg.controlMode.state = nasa_r2_common_msgs::JointControlMode::OFF;
    jointControlManagerMap["/r2/left_leg/joint0"]->setCommandStates(commandMsg);
    jointControlManagerMap["/r2/right_leg/gripper/joint0"]->setCommandStates(commandMsg);
    jointControlManagerMap["/r2/left_arm/joint0"]->setCommandStates(commandMsg);
    jointControlManagerMap["/r2/neck/joint0"]->setCommandStates(commandMsg);
    jointControlManagerMap["/r2/right_arm/wrist"]->setCommandStates(commandMsg);
    commandMsg.controlMode.state = nasa_r2_common_msgs::JointControlMode::PARK;
    jointControlManagerMap["/r2/left_leg/joint0"]->setCommandStates(commandMsg);
    jointControlManagerMap["/r2/right_leg/gripper/joint0"]->setCommandStates(commandMsg);
    jointControlManagerMap["/r2/left_arm/joint0"]->setCommandStates(commandMsg);
    jointControlManagerMap["/r2/neck/joint0"]->setCommandStates(commandMsg);
    jointControlManagerMap["/r2/right_arm/wrist"]->setCommandStates(commandMsg);

    nodeRegisterManager->nodeRegisterMap["/r2/left_leg/joint0"].status["/r2/left_leg/joint0/StatReg1"].registerValue                   = 0x9011; // 1001 0000 0001 0001
    nodeRegisterManager->nodeRegisterMap["/r2/right_leg/gripper/joint0"].status["/r2/right_leg/gripper/joint0/StatReg1"].registerValue = 0x9011; // 1001 0000 0001 0001
    nodeRegisterManager->nodeRegisterMap["/r2/left_arm/joint0"].status["/r2/left_arm/joint0/StatReg1"].registerValue                   = 0x9011; // 1001 0000 0001 0001
    nodeRegisterManager->nodeRegisterMap["/r2/neck/joint0"].status["/r2/neck/joint0/StatReg1"].registerValue                           = 0x9011; // 1001 0000 0001 0001
    io.setLiveCoeff(StringUtilities::makeFullyQualifiedRoboDynElement("/r2/right_arm/wrist", "CalibrationState"), nasa_r2_common_msgs::JointControlCalibrationMode::DISABLE);

    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::PARK, jointControlManagerMap["/r2/left_leg/joint0"]->getActualStates().controlMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::PARK, jointControlManagerMap["/r2/right_leg/gripper/joint0"]->getActualStates().controlMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::PARK, jointControlManagerMap["/r2/left_arm/joint0"]->getActualStates().controlMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::PARK, jointControlManagerMap["/r2/neck/joint0"]->getActualStates().controlMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::PARK, jointControlManagerMap["/r2/right_arm/wrist"]->getActualStates().controlMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::PARK, jointControlManagerMap["/r2/left_leg/joint0"]->getCommandStates().controlMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::PARK, jointControlManagerMap["/r2/right_leg/gripper/joint0"]->getCommandStates().controlMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::PARK, jointControlManagerMap["/r2/left_arm/joint0"]->getCommandStates().controlMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::PARK, jointControlManagerMap["/r2/neck/joint0"]->getCommandStates().controlMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::PARK, jointControlManagerMap["/r2/right_arm/wrist"]->getCommandStates().controlMode.state);

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

    // Can't go back into BOOTLOADER
    commandMsg.controlMode.state = nasa_r2_common_msgs::JointControlMode::BOOTLOADER;
    jointControlManagerMap["/r2/left_leg/joint0"]->setCommandStates(commandMsg);
    jointControlManagerMap["/r2/right_leg/gripper/joint0"]->setCommandStates(commandMsg);
    jointControlManagerMap["/r2/left_arm/joint0"]->setCommandStates(commandMsg);
    jointControlManagerMap["/r2/neck/joint0"]->setCommandStates(commandMsg);
    jointControlManagerMap["/r2/right_arm/wrist"]->setCommandStates(commandMsg);

    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::PARK, jointControlManagerMap["/r2/left_leg/joint0"]->getActualStates().controlMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::PARK, jointControlManagerMap["/r2/right_leg/gripper/joint0"]->getActualStates().controlMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::PARK, jointControlManagerMap["/r2/left_arm/joint0"]->getActualStates().controlMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::PARK, jointControlManagerMap["/r2/neck/joint0"]->getActualStates().controlMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::PARK, jointControlManagerMap["/r2/right_arm/wrist"]->getActualStates().controlMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::PARK, jointControlManagerMap["/r2/left_leg/joint0"]->getCommandStates().controlMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::PARK, jointControlManagerMap["/r2/right_leg/gripper/joint0"]->getCommandStates().controlMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::PARK, jointControlManagerMap["/r2/left_arm/joint0"]->getCommandStates().controlMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::PARK, jointControlManagerMap["/r2/neck/joint0"]->getCommandStates().controlMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::PARK, jointControlManagerMap["/r2/right_arm/wrist"]->getCommandStates().controlMode.state);

    // Cant get back to OFF
    commandMsg.controlMode.state = nasa_r2_common_msgs::JointControlMode::OFF;
    jointControlManagerMap["/r2/left_leg/joint0"]->setCommandStates(commandMsg);
    jointControlManagerMap["/r2/right_leg/gripper/joint0"]->setCommandStates(commandMsg);
    jointControlManagerMap["/r2/left_arm/joint0"]->setCommandStates(commandMsg);
    jointControlManagerMap["/r2/neck/joint0"]->setCommandStates(commandMsg);
    jointControlManagerMap["/r2/right_arm/wrist"]->setCommandStates(commandMsg);

    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::PARK, jointControlManagerMap["/r2/left_leg/joint0"]->getActualStates().controlMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::PARK, jointControlManagerMap["/r2/right_leg/gripper/joint0"]->getActualStates().controlMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::PARK, jointControlManagerMap["/r2/left_arm/joint0"]->getActualStates().controlMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::PARK, jointControlManagerMap["/r2/neck/joint0"]->getActualStates().controlMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::OFF, jointControlManagerMap["/r2/right_arm/wrist"]->getActualStates().controlMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::PARK, jointControlManagerMap["/r2/left_leg/joint0"]->getCommandStates().controlMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::PARK, jointControlManagerMap["/r2/right_leg/gripper/joint0"]->getCommandStates().controlMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::PARK, jointControlManagerMap["/r2/left_arm/joint0"]->getCommandStates().controlMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::PARK, jointControlManagerMap["/r2/neck/joint0"]->getCommandStates().controlMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::OFF, jointControlManagerMap["/r2/right_arm/wrist"]->getCommandStates().controlMode.state);
}

TEST_F(JointControlManagerTest, Neutral)
{
    // Can't NEUTRAL from BOOTLOADER (SEA)
    commandMsg.controlMode.state = nasa_r2_common_msgs::JointControlMode::NEUTRAL;
    jointControlManagerMap["/r2/left_leg/joint0"]->setCommandStates(commandMsg);
    jointControlManagerMap["/r2/right_leg/gripper/joint0"]->setCommandStates(commandMsg);
    jointControlManagerMap["/r2/left_arm/joint0"]->setCommandStates(commandMsg);
    jointControlManagerMap["/r2/neck/joint0"]->setCommandStates(commandMsg);
    jointControlManagerMap["/r2/right_arm/wrist"]->setCommandStates(commandMsg);

    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::BOOTLOADER, jointControlManagerMap["/r2/left_leg/joint0"]->getActualStates().controlMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::BOOTLOADER, jointControlManagerMap["/r2/right_leg/gripper/joint0"]->getActualStates().controlMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::BOOTLOADER, jointControlManagerMap["/r2/left_arm/joint0"]->getActualStates().controlMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::BOOTLOADER, jointControlManagerMap["/r2/neck/joint0"]->getActualStates().controlMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::NEUTRAL,    jointControlManagerMap["/r2/right_arm/wrist"]->getActualStates().controlMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::BOOTLOADER, jointControlManagerMap["/r2/left_leg/joint0"]->getCommandStates().controlMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::BOOTLOADER, jointControlManagerMap["/r2/right_leg/gripper/joint0"]->getCommandStates().controlMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::BOOTLOADER, jointControlManagerMap["/r2/left_arm/joint0"]->getCommandStates().controlMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::BOOTLOADER, jointControlManagerMap["/r2/neck/joint0"]->getCommandStates().controlMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::NEUTRAL,    jointControlManagerMap["/r2/right_arm/wrist"]->getCommandStates().controlMode.state);

    // Can NEUTRAL through OFF
    commandMsg.controlMode.state = nasa_r2_common_msgs::JointControlMode::OFF;
    jointControlManagerMap["/r2/left_leg/joint0"]->setCommandStates(commandMsg);
    jointControlManagerMap["/r2/right_leg/gripper/joint0"]->setCommandStates(commandMsg);
    jointControlManagerMap["/r2/left_arm/joint0"]->setCommandStates(commandMsg);
    jointControlManagerMap["/r2/neck/joint0"]->setCommandStates(commandMsg);
    jointControlManagerMap["/r2/right_arm/wrist"]->setCommandStates(commandMsg);
    commandMsg.controlMode.state = nasa_r2_common_msgs::JointControlMode::NEUTRAL;
    jointControlManagerMap["/r2/left_leg/joint0"]->setCommandStates(commandMsg);
    jointControlManagerMap["/r2/right_leg/gripper/joint0"]->setCommandStates(commandMsg);
    jointControlManagerMap["/r2/left_arm/joint0"]->setCommandStates(commandMsg);
    jointControlManagerMap["/r2/neck/joint0"]->setCommandStates(commandMsg);
    jointControlManagerMap["/r2/right_arm/wrist"]->setCommandStates(commandMsg);

    nodeRegisterManager->nodeRegisterMap["/r2/left_leg/joint0"].status["/r2/left_leg/joint0/StatReg1"].registerValue                   = 0x9012; // 1001 0000 0001 0010
    nodeRegisterManager->nodeRegisterMap["/r2/right_leg/gripper/joint0"].status["/r2/right_leg/gripper/joint0/StatReg1"].registerValue = 0x9012; // 1001 0000 0001 0010
    nodeRegisterManager->nodeRegisterMap["/r2/left_arm/joint0"].status["/r2/left_arm/joint0/StatReg1"].registerValue                   = 0x9012; // 1001 0000 0001 0010
    nodeRegisterManager->nodeRegisterMap["/r2/neck/joint0"].status["/r2/neck/joint0/StatReg1"].registerValue                           = 0x9012; // 1001 0000 0001 0010
    io.setLiveCoeff(StringUtilities::makeFullyQualifiedRoboDynElement("/r2/right_arm/wrist", "CalibrationState"), nasa_r2_common_msgs::JointControlCalibrationMode::DISABLE);

    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::NEUTRAL, jointControlManagerMap["/r2/left_leg/joint0"]->getActualStates().controlMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::NEUTRAL, jointControlManagerMap["/r2/right_leg/gripper/joint0"]->getActualStates().controlMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::NEUTRAL, jointControlManagerMap["/r2/left_arm/joint0"]->getActualStates().controlMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::NEUTRAL, jointControlManagerMap["/r2/neck/joint0"]->getActualStates().controlMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::NEUTRAL, jointControlManagerMap["/r2/right_arm/wrist"]->getActualStates().controlMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::NEUTRAL, jointControlManagerMap["/r2/left_leg/joint0"]->getCommandStates().controlMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::NEUTRAL, jointControlManagerMap["/r2/right_leg/gripper/joint0"]->getCommandStates().controlMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::NEUTRAL, jointControlManagerMap["/r2/left_arm/joint0"]->getCommandStates().controlMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::NEUTRAL, jointControlManagerMap["/r2/neck/joint0"]->getCommandStates().controlMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::NEUTRAL, jointControlManagerMap["/r2/right_arm/wrist"]->getCommandStates().controlMode.state);

    EXPECT_EQ(1, nodeRegisterManager->getControlValue("/r2/left_leg/joint0", "BootEnable"));
    EXPECT_EQ(1, nodeRegisterManager->getControlValue("/r2/left_leg/joint0", "BridgeEnable"));
    EXPECT_EQ(0, nodeRegisterManager->getControlValue("/r2/left_leg/joint0", "MotorEnable"));

    EXPECT_EQ(1, nodeRegisterManager->getControlValue("/r2/right_leg/gripper/joint0", "BootEnable"));
    EXPECT_EQ(1, nodeRegisterManager->getControlValue("/r2/right_leg/gripper/joint0", "BridgeEnable"));
    EXPECT_EQ(0, nodeRegisterManager->getControlValue("/r2/right_leg/gripper/joint0", "MotorEnable"));

    EXPECT_EQ(1, nodeRegisterManager->getControlValue("/r2/left_arm/joint0", "BootEnable"));
    EXPECT_EQ(1, nodeRegisterManager->getControlValue("/r2/left_arm/joint0", "BridgeEnable"));
    EXPECT_EQ(0, nodeRegisterManager->getControlValue("/r2/left_arm/joint0", "MotorEnable"));
    EXPECT_EQ(1, nodeRegisterManager->getControlValue("/r2/left_arm/joint0", "BrakeRelease"));

    EXPECT_EQ(1, nodeRegisterManager->getControlValue("/r2/neck/joint0", "BootEnable"));
    EXPECT_EQ(1, nodeRegisterManager->getControlValue("/r2/neck/joint0", "BridgeEnable"));
    EXPECT_EQ(0, nodeRegisterManager->getControlValue("/r2/neck/joint0", "MotorEnable"));
    EXPECT_EQ(1, nodeRegisterManager->getControlValue("/r2/neck/joint0", "BrakeRelease"));

    // Can go to PARK
    commandMsg.controlMode.state = nasa_r2_common_msgs::JointControlMode::PARK;
    jointControlManagerMap["/r2/left_leg/joint0"]->setCommandStates(commandMsg);
    jointControlManagerMap["/r2/right_leg/gripper/joint0"]->setCommandStates(commandMsg);
    jointControlManagerMap["/r2/left_arm/joint0"]->setCommandStates(commandMsg);
    jointControlManagerMap["/r2/neck/joint0"]->setCommandStates(commandMsg);
    jointControlManagerMap["/r2/right_arm/wrist"]->setCommandStates(commandMsg);

    nodeRegisterManager->nodeRegisterMap["/r2/left_leg/joint0"].status["/r2/left_leg/joint0/StatReg1"].registerValue                   = 0x9011; // 1001 0000 0001 0001
    nodeRegisterManager->nodeRegisterMap["/r2/right_leg/gripper/joint0"].status["/r2/right_leg/gripper/joint0/StatReg1"].registerValue = 0x9011; // 1001 0000 0001 0001
    nodeRegisterManager->nodeRegisterMap["/r2/left_arm/joint0"].status["/r2/left_arm/joint0/StatReg1"].registerValue                   = 0x9011; // 1001 0000 0001 0001
    nodeRegisterManager->nodeRegisterMap["/r2/neck/joint0"].status["/r2/neck/joint0/StatReg1"].registerValue                           = 0x9011; // 1001 0000 0001 0001

    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::PARK, jointControlManagerMap["/r2/left_leg/joint0"]->getActualStates().controlMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::PARK, jointControlManagerMap["/r2/right_leg/gripper/joint0"]->getActualStates().controlMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::PARK, jointControlManagerMap["/r2/left_arm/joint0"]->getActualStates().controlMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::PARK, jointControlManagerMap["/r2/neck/joint0"]->getActualStates().controlMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::PARK, jointControlManagerMap["/r2/right_arm/wrist"]->getActualStates().controlMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::PARK, jointControlManagerMap["/r2/left_leg/joint0"]->getCommandStates().controlMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::PARK, jointControlManagerMap["/r2/right_leg/gripper/joint0"]->getCommandStates().controlMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::PARK, jointControlManagerMap["/r2/left_arm/joint0"]->getCommandStates().controlMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::PARK, jointControlManagerMap["/r2/neck/joint0"]->getCommandStates().controlMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::PARK, jointControlManagerMap["/r2/right_arm/wrist"]->getCommandStates().controlMode.state);

    // Can NEUTRAL from PARK
    commandMsg.controlMode.state = nasa_r2_common_msgs::JointControlMode::NEUTRAL;
    jointControlManagerMap["/r2/left_leg/joint0"]->setCommandStates(commandMsg);
    jointControlManagerMap["/r2/right_leg/gripper/joint0"]->setCommandStates(commandMsg);
    jointControlManagerMap["/r2/left_arm/joint0"]->setCommandStates(commandMsg);
    jointControlManagerMap["/r2/neck/joint0"]->setCommandStates(commandMsg);
    jointControlManagerMap["/r2/right_arm/wrist"]->setCommandStates(commandMsg);

    nodeRegisterManager->nodeRegisterMap["/r2/left_leg/joint0"].status["/r2/left_leg/joint0/StatReg1"].registerValue                   = 0x9012; // 1001 0000 0001 0010
    nodeRegisterManager->nodeRegisterMap["/r2/right_leg/gripper/joint0"].status["/r2/right_leg/gripper/joint0/StatReg1"].registerValue = 0x9012; // 1001 0000 0001 0010
    nodeRegisterManager->nodeRegisterMap["/r2/left_arm/joint0"].status["/r2/left_arm/joint0/StatReg1"].registerValue                   = 0x9012; // 1001 0000 0001 0010
    nodeRegisterManager->nodeRegisterMap["/r2/neck/joint0"].status["/r2/neck/joint0/StatReg1"].registerValue                           = 0x9012; // 1001 0000 0001 0010

    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::NEUTRAL, jointControlManagerMap["/r2/left_leg/joint0"]->getActualStates().controlMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::NEUTRAL, jointControlManagerMap["/r2/right_leg/gripper/joint0"]->getActualStates().controlMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::NEUTRAL, jointControlManagerMap["/r2/left_arm/joint0"]->getActualStates().controlMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::NEUTRAL, jointControlManagerMap["/r2/neck/joint0"]->getActualStates().controlMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::NEUTRAL, jointControlManagerMap["/r2/right_arm/wrist"]->getActualStates().controlMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::NEUTRAL, jointControlManagerMap["/r2/left_leg/joint0"]->getCommandStates().controlMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::NEUTRAL, jointControlManagerMap["/r2/right_leg/gripper/joint0"]->getCommandStates().controlMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::NEUTRAL, jointControlManagerMap["/r2/left_arm/joint0"]->getCommandStates().controlMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::NEUTRAL, jointControlManagerMap["/r2/neck/joint0"]->getCommandStates().controlMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::NEUTRAL, jointControlManagerMap["/r2/right_arm/wrist"]->getCommandStates().controlMode.state);

    // Can't OFF from NEUTRAL
    commandMsg.controlMode.state = nasa_r2_common_msgs::JointControlMode::OFF;
    jointControlManagerMap["/r2/left_leg/joint0"]->setCommandStates(commandMsg);
    jointControlManagerMap["/r2/right_leg/gripper/joint0"]->setCommandStates(commandMsg);
    jointControlManagerMap["/r2/left_arm/joint0"]->setCommandStates(commandMsg);
    jointControlManagerMap["/r2/neck/joint0"]->setCommandStates(commandMsg);
    jointControlManagerMap["/r2/right_arm/wrist"]->setCommandStates(commandMsg);

    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::NEUTRAL, jointControlManagerMap["/r2/left_leg/joint0"]->getActualStates().controlMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::NEUTRAL, jointControlManagerMap["/r2/right_leg/gripper/joint0"]->getActualStates().controlMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::NEUTRAL, jointControlManagerMap["/r2/left_arm/joint0"]->getActualStates().controlMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::NEUTRAL, jointControlManagerMap["/r2/neck/joint0"]->getActualStates().controlMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::OFF, jointControlManagerMap["/r2/right_arm/wrist"]->getActualStates().controlMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::NEUTRAL, jointControlManagerMap["/r2/left_leg/joint0"]->getCommandStates().controlMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::NEUTRAL, jointControlManagerMap["/r2/right_leg/gripper/joint0"]->getCommandStates().controlMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::NEUTRAL, jointControlManagerMap["/r2/left_arm/joint0"]->getCommandStates().controlMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::NEUTRAL, jointControlManagerMap["/r2/neck/joint0"]->getCommandStates().controlMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::OFF, jointControlManagerMap["/r2/right_arm/wrist"]->getCommandStates().controlMode.state);
}

TEST_F(JointControlManagerTest, Drive)
{
    // Can't DRIVE from BOOTLOADER (SEA)
    commandMsg.controlMode.state = nasa_r2_common_msgs::JointControlMode::DRIVE;
    jointControlManagerMap["/r2/left_leg/joint0"]->setCommandStates(commandMsg);
    jointControlManagerMap["/r2/right_leg/gripper/joint0"]->setCommandStates(commandMsg);
    jointControlManagerMap["/r2/left_arm/joint0"]->setCommandStates(commandMsg);
    jointControlManagerMap["/r2/neck/joint0"]->setCommandStates(commandMsg);
    jointControlManagerMap["/r2/right_arm/wrist"]->setCommandStates(commandMsg);

    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::BOOTLOADER, jointControlManagerMap["/r2/left_leg/joint0"]->getActualStates().controlMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::BOOTLOADER, jointControlManagerMap["/r2/right_leg/gripper/joint0"]->getActualStates().controlMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::BOOTLOADER, jointControlManagerMap["/r2/left_arm/joint0"]->getActualStates().controlMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::BOOTLOADER, jointControlManagerMap["/r2/neck/joint0"]->getActualStates().controlMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::BOOTLOADER, jointControlManagerMap["/r2/right_arm/wrist"]->getActualStates().controlMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::BOOTLOADER, jointControlManagerMap["/r2/left_leg/joint0"]->getCommandStates().controlMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::BOOTLOADER, jointControlManagerMap["/r2/right_leg/gripper/joint0"]->getCommandStates().controlMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::BOOTLOADER, jointControlManagerMap["/r2/left_arm/joint0"]->getCommandStates().controlMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::BOOTLOADER, jointControlManagerMap["/r2/neck/joint0"]->getCommandStates().controlMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::BOOTLOADER, jointControlManagerMap["/r2/right_arm/wrist"]->getCommandStates().controlMode.state);

    // Can DRIVE through PARK
    commandMsg.controlMode.state = nasa_r2_common_msgs::JointControlMode::OFF;
    jointControlManagerMap["/r2/left_leg/joint0"]->setCommandStates(commandMsg);
    jointControlManagerMap["/r2/right_leg/gripper/joint0"]->setCommandStates(commandMsg);
    jointControlManagerMap["/r2/left_arm/joint0"]->setCommandStates(commandMsg);
    jointControlManagerMap["/r2/neck/joint0"]->setCommandStates(commandMsg);
    jointControlManagerMap["/r2/right_arm/wrist"]->setCommandStates(commandMsg);
    
    commandMsg.controlMode.state = nasa_r2_common_msgs::JointControlMode::PARK;
    jointControlManagerMap["/r2/left_leg/joint0"]->setCommandStates(commandMsg);
    jointControlManagerMap["/r2/right_leg/gripper/joint0"]->setCommandStates(commandMsg);
    jointControlManagerMap["/r2/left_arm/joint0"]->setCommandStates(commandMsg);
    jointControlManagerMap["/r2/neck/joint0"]->setCommandStates(commandMsg);
    jointControlManagerMap["/r2/right_arm/wrist"]->setCommandStates(commandMsg);
    io.setLiveCoeff(StringUtilities::makeFullyQualifiedRoboDynElement("/r2/right_arm/wrist", "CalibrationMode"), nasa_r2_common_msgs::JointControlCalibrationMode::DISABLE);
    nodeRegisterManager->nodeRegisterMap["/r2/left_leg/joint0"].status["/r2/left_leg/joint0/StatReg1"].registerValue                   = 0x9011; // 1001 0000 0001 0001
    nodeRegisterManager->nodeRegisterMap["/r2/right_leg/gripper/joint0"].status["/r2/right_leg/gripper/joint0/StatReg1"].registerValue = 0x9011; // 1001 0000 0001 0001
    nodeRegisterManager->nodeRegisterMap["/r2/left_arm/joint0"].status["/r2/left_arm/joint0/StatReg1"].registerValue                   = 0x9011; // 1001 0000 0001 0001
    nodeRegisterManager->nodeRegisterMap["/r2/neck/joint0"].status["/r2/neck/joint0/StatReg1"].registerValue                           = 0x9011; // 1001 0000 0001 0001

    commandMsg.controlMode.state = nasa_r2_common_msgs::JointControlMode::DRIVE;
    jointControlManagerMap["/r2/left_leg/joint0"]->setCommandStates(commandMsg);
    jointControlManagerMap["/r2/right_leg/gripper/joint0"]->setCommandStates(commandMsg);
    jointControlManagerMap["/r2/left_arm/joint0"]->setCommandStates(commandMsg);
    jointControlManagerMap["/r2/neck/joint0"]->setCommandStates(commandMsg);
    jointControlManagerMap["/r2/right_arm/wrist"]->setCommandStates(commandMsg);
    nodeRegisterManager->nodeRegisterMap["/r2/left_leg/joint0"].status["/r2/left_leg/joint0/StatReg1"].registerValue                   = 0x9013; // 1001 0000 0001 0011
    nodeRegisterManager->nodeRegisterMap["/r2/right_leg/gripper/joint0"].status["/r2/right_leg/gripper/joint0/StatReg1"].registerValue = 0x9013; // 1001 0000 0001 0011
    nodeRegisterManager->nodeRegisterMap["/r2/left_arm/joint0"].status["/r2/left_arm/joint0/StatReg1"].registerValue                   = 0x9013; // 1001 0000 0001 0011
    nodeRegisterManager->nodeRegisterMap["/r2/neck/joint0"].status["/r2/neck/joint0/StatReg1"].registerValue                           = 0x9013; // 1001 0000 0001 0011

    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::DRIVE, jointControlManagerMap["/r2/left_leg/joint0"]->getActualStates().controlMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::DRIVE, jointControlManagerMap["/r2/right_leg/gripper/joint0"]->getActualStates().controlMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::DRIVE, jointControlManagerMap["/r2/left_arm/joint0"]->getActualStates().controlMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::DRIVE, jointControlManagerMap["/r2/neck/joint0"]->getActualStates().controlMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::DRIVE, jointControlManagerMap["/r2/right_arm/wrist"]->getActualStates().controlMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::DRIVE, jointControlManagerMap["/r2/left_leg/joint0"]->getCommandStates().controlMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::DRIVE, jointControlManagerMap["/r2/right_leg/gripper/joint0"]->getCommandStates().controlMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::DRIVE, jointControlManagerMap["/r2/left_arm/joint0"]->getCommandStates().controlMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::DRIVE, jointControlManagerMap["/r2/neck/joint0"]->getCommandStates().controlMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::DRIVE, jointControlManagerMap["/r2/right_arm/wrist"]->getCommandStates().controlMode.state);

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

    // Can PARK from DRIVE
    commandMsg.controlMode.state = nasa_r2_common_msgs::JointControlMode::PARK;
    jointControlManagerMap["/r2/left_leg/joint0"]->setCommandStates(commandMsg);
    jointControlManagerMap["/r2/right_leg/gripper/joint0"]->setCommandStates(commandMsg);
    jointControlManagerMap["/r2/left_arm/joint0"]->setCommandStates(commandMsg);
    jointControlManagerMap["/r2/neck/joint0"]->setCommandStates(commandMsg);
    jointControlManagerMap["/r2/right_arm/wrist"]->setCommandStates(commandMsg);
    nodeRegisterManager->nodeRegisterMap["/r2/left_leg/joint0"].status["/r2/left_leg/joint0/StatReg1"].registerValue                   = 0x9011; // 1001 0000 0001 0001
    nodeRegisterManager->nodeRegisterMap["/r2/right_leg/gripper/joint0"].status["/r2/right_leg/gripper/joint0/StatReg1"].registerValue = 0x9011; // 1001 0000 0001 0001
    nodeRegisterManager->nodeRegisterMap["/r2/left_arm/joint0"].status["/r2/left_arm/joint0/StatReg1"].registerValue                   = 0x9011; // 1001 0000 0001 0001
    nodeRegisterManager->nodeRegisterMap["/r2/neck/joint0"].status["/r2/neck/joint0/StatReg1"].registerValue                           = 0x9011; // 1001 0000 0001 0001

    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::PARK, jointControlManagerMap["/r2/left_leg/joint0"]->getActualStates().controlMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::PARK, jointControlManagerMap["/r2/right_leg/gripper/joint0"]->getActualStates().controlMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::PARK, jointControlManagerMap["/r2/left_arm/joint0"]->getActualStates().controlMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::PARK, jointControlManagerMap["/r2/neck/joint0"]->getActualStates().controlMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::PARK, jointControlManagerMap["/r2/right_arm/wrist"]->getActualStates().controlMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::PARK, jointControlManagerMap["/r2/left_leg/joint0"]->getCommandStates().controlMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::PARK, jointControlManagerMap["/r2/right_leg/gripper/joint0"]->getCommandStates().controlMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::PARK, jointControlManagerMap["/r2/left_arm/joint0"]->getCommandStates().controlMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::PARK, jointControlManagerMap["/r2/neck/joint0"]->getCommandStates().controlMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::PARK, jointControlManagerMap["/r2/right_arm/wrist"]->getCommandStates().controlMode.state);

    // Can't NEUTRAL from DRIVE
    commandMsg.controlMode.state = nasa_r2_common_msgs::JointControlMode::DRIVE;
    jointControlManagerMap["/r2/left_leg/joint0"]->setCommandStates(commandMsg);
    jointControlManagerMap["/r2/right_leg/gripper/joint0"]->setCommandStates(commandMsg);
    jointControlManagerMap["/r2/left_arm/joint0"]->setCommandStates(commandMsg);
    jointControlManagerMap["/r2/neck/joint0"]->setCommandStates(commandMsg);
    jointControlManagerMap["/r2/right_arm/wrist"]->setCommandStates(commandMsg);
    nodeRegisterManager->nodeRegisterMap["/r2/left_leg/joint0"].status["/r2/left_leg/joint0/StatReg1"].registerValue                   = 0x9013; // 1001 0000 0001 0011
    nodeRegisterManager->nodeRegisterMap["/r2/right_leg/gripper/joint0"].status["/r2/right_leg/gripper/joint0/StatReg1"].registerValue = 0x9013; // 1001 0000 0001 0011
    nodeRegisterManager->nodeRegisterMap["/r2/left_arm/joint0"].status["/r2/left_arm/joint0/StatReg1"].registerValue                   = 0x9013; // 1001 0000 0001 0011
    nodeRegisterManager->nodeRegisterMap["/r2/neck/joint0"].status["/r2/neck/joint0/StatReg1"].registerValue                           = 0x9013; // 1001 0000 0001 0011
    
    commandMsg.controlMode.state = nasa_r2_common_msgs::JointControlMode::NEUTRAL;
    jointControlManagerMap["/r2/left_leg/joint0"]->setCommandStates(commandMsg);
    jointControlManagerMap["/r2/right_leg/gripper/joint0"]->setCommandStates(commandMsg);
    jointControlManagerMap["/r2/left_arm/joint0"]->setCommandStates(commandMsg);
    jointControlManagerMap["/r2/neck/joint0"]->setCommandStates(commandMsg);
    jointControlManagerMap["/r2/right_arm/wrist"]->setCommandStates(commandMsg);

    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::DRIVE, jointControlManagerMap["/r2/left_leg/joint0"]->getActualStates().controlMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::DRIVE, jointControlManagerMap["/r2/right_leg/gripper/joint0"]->getActualStates().controlMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::DRIVE, jointControlManagerMap["/r2/left_arm/joint0"]->getActualStates().controlMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::DRIVE, jointControlManagerMap["/r2/neck/joint0"]->getActualStates().controlMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::NEUTRAL, jointControlManagerMap["/r2/right_arm/wrist"]->getActualStates().controlMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::DRIVE, jointControlManagerMap["/r2/left_leg/joint0"]->getCommandStates().controlMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::DRIVE, jointControlManagerMap["/r2/right_leg/gripper/joint0"]->getCommandStates().controlMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::DRIVE, jointControlManagerMap["/r2/left_arm/joint0"]->getCommandStates().controlMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::DRIVE, jointControlManagerMap["/r2/neck/joint0"]->getCommandStates().controlMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::NEUTRAL, jointControlManagerMap["/r2/right_arm/wrist"]->getCommandStates().controlMode.state);

    // Can't OFF from DRIVE
    commandMsg.controlMode.state = nasa_r2_common_msgs::JointControlMode::DRIVE;
    jointControlManagerMap["/r2/left_leg/joint0"]->setCommandStates(commandMsg);
    jointControlManagerMap["/r2/right_leg/gripper/joint0"]->setCommandStates(commandMsg);
    jointControlManagerMap["/r2/left_arm/joint0"]->setCommandStates(commandMsg);
    jointControlManagerMap["/r2/neck/joint0"]->setCommandStates(commandMsg);
    jointControlManagerMap["/r2/right_arm/wrist"]->setCommandStates(commandMsg);
    
    commandMsg.controlMode.state = nasa_r2_common_msgs::JointControlMode::OFF;
    jointControlManagerMap["/r2/left_leg/joint0"]->setCommandStates(commandMsg);
    jointControlManagerMap["/r2/right_leg/gripper/joint0"]->setCommandStates(commandMsg);
    jointControlManagerMap["/r2/left_arm/joint0"]->setCommandStates(commandMsg);
    jointControlManagerMap["/r2/neck/joint0"]->setCommandStates(commandMsg);
    jointControlManagerMap["/r2/right_arm/wrist"]->setCommandStates(commandMsg);

    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::DRIVE, jointControlManagerMap["/r2/left_leg/joint0"]->getActualStates().controlMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::DRIVE, jointControlManagerMap["/r2/right_leg/gripper/joint0"]->getActualStates().controlMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::DRIVE, jointControlManagerMap["/r2/left_arm/joint0"]->getActualStates().controlMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::DRIVE, jointControlManagerMap["/r2/neck/joint0"]->getActualStates().controlMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::OFF, jointControlManagerMap["/r2/right_arm/wrist"]->getActualStates().controlMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::DRIVE, jointControlManagerMap["/r2/left_leg/joint0"]->getCommandStates().controlMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::DRIVE, jointControlManagerMap["/r2/right_leg/gripper/joint0"]->getCommandStates().controlMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::DRIVE, jointControlManagerMap["/r2/left_arm/joint0"]->getCommandStates().controlMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::DRIVE, jointControlManagerMap["/r2/neck/joint0"]->getCommandStates().controlMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::OFF, jointControlManagerMap["/r2/right_arm/wrist"]->getCommandStates().controlMode.state);
}

TEST_F(JointControlManagerTest, ClearFault)
{
    // Get ready
    commandMsg.controlMode.state = nasa_r2_common_msgs::JointControlMode::OFF;
    jointControlManagerMap["/r2/left_leg/joint0"]->setCommandStates(commandMsg);
    jointControlManagerMap["/r2/right_leg/gripper/joint0"]->setCommandStates(commandMsg);
    jointControlManagerMap["/r2/left_arm/joint0"]->setCommandStates(commandMsg);
    jointControlManagerMap["/r2/neck/joint0"]->setCommandStates(commandMsg);
    jointControlManagerMap["/r2/right_arm/wrist"]->setCommandStates(commandMsg);
    commandMsg.controlMode.state = nasa_r2_common_msgs::JointControlMode::PARK;
    jointControlManagerMap["/r2/left_leg/joint0"]->setCommandStates(commandMsg);
    jointControlManagerMap["/r2/right_leg/gripper/joint0"]->setCommandStates(commandMsg);
    jointControlManagerMap["/r2/left_arm/joint0"]->setCommandStates(commandMsg);
    jointControlManagerMap["/r2/neck/joint0"]->setCommandStates(commandMsg);
    jointControlManagerMap["/r2/right_arm/wrist"]->setCommandStates(commandMsg);

    nodeRegisterManager->nodeRegisterMap["/r2/left_leg/joint0"].status["/r2/left_leg/joint0/StatReg1"].registerValue                   = 0x9011; // 1001 0000 0001 0001
    nodeRegisterManager->nodeRegisterMap["/r2/right_leg/gripper/joint0"].status["/r2/right_leg/gripper/joint0/StatReg1"].registerValue = 0x9011; // 1001 0000 0001 0001
    nodeRegisterManager->nodeRegisterMap["/r2/left_arm/joint0"].status["/r2/left_arm/joint0/StatReg1"].registerValue                   = 0x9011; // 1001 0000 0001 0001
    nodeRegisterManager->nodeRegisterMap["/r2/neck/joint0"].status["/r2/neck/joint0/StatReg1"].registerValue                           = 0x9011; // 1001 0000 0001 0001
    io.setLiveCoeff(StringUtilities::makeFullyQualifiedRoboDynElement("/r2/right_arm/wrist", "CalibrationState"), nasa_r2_common_msgs::JointControlCalibrationMode::DISABLE);

    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::PARK, jointControlManagerMap["/r2/left_leg/joint0"]->getActualStates().controlMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::PARK, jointControlManagerMap["/r2/right_leg/gripper/joint0"]->getActualStates().controlMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::PARK, jointControlManagerMap["/r2/left_arm/joint0"]->getActualStates().controlMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::PARK, jointControlManagerMap["/r2/neck/joint0"]->getActualStates().controlMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::PARK, jointControlManagerMap["/r2/right_arm/wrist"]->getActualStates().controlMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::PARK, jointControlManagerMap["/r2/left_leg/joint0"]->getCommandStates().controlMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::PARK, jointControlManagerMap["/r2/right_leg/gripper/joint0"]->getCommandStates().controlMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::PARK, jointControlManagerMap["/r2/left_arm/joint0"]->getCommandStates().controlMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::PARK, jointControlManagerMap["/r2/neck/joint0"]->getCommandStates().controlMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::PARK, jointControlManagerMap["/r2/right_arm/wrist"]->getCommandStates().controlMode.state);

    commandMsg.calibrationMode.state = nasa_r2_common_msgs::JointControlCalibrationMode::ENABLE;
    jointControlManagerMap["/r2/left_leg/joint0"]->setCommandStates(commandMsg);
    jointControlManagerMap["/r2/right_leg/gripper/joint0"]->setCommandStates(commandMsg);
    jointControlManagerMap["/r2/left_arm/joint0"]->setCommandStates(commandMsg);
    jointControlManagerMap["/r2/neck/joint0"]->setCommandStates(commandMsg);
    jointControlManagerMap["/r2/right_arm/wrist"]->setCommandStates(commandMsg);
    io.setLiveCoeff(StringUtilities::makeFullyQualifiedRoboDynElement("/r2/right_arm/wrist", "CalibrationMode"), nasa_r2_common_msgs::JointControlCalibrationMode::DISABLE);
    commandMsg.controlMode.state = nasa_r2_common_msgs::JointControlMode::DRIVE;
    commandMsg.calibrationMode.state = nasa_r2_common_msgs::JointControlCalibrationMode::IGNORE;
    jointControlManagerMap["/r2/left_leg/joint0"]->setCommandStates(commandMsg);
    jointControlManagerMap["/r2/right_leg/gripper/joint0"]->setCommandStates(commandMsg);
    jointControlManagerMap["/r2/left_arm/joint0"]->setCommandStates(commandMsg);
    jointControlManagerMap["/r2/neck/joint0"]->setCommandStates(commandMsg);
    jointControlManagerMap["/r2/right_arm/wrist"]->setCommandStates(commandMsg);

    // Fake MotComSource == 1, BridgeEnabled, MotorEnabled, BrakeReleased
    nodeRegisterManager->nodeRegisterMap["/r2/left_leg/joint0"].status["/r2/left_leg/joint0/StatReg1"].registerValue                   = 0x9053; // 1001 0000 0101 0011
    nodeRegisterManager->nodeRegisterMap["/r2/right_leg/gripper/joint0"].status["/r2/right_leg/gripper/joint0/StatReg1"].registerValue = 0x9053; // 1001 0000 0101 0011
    nodeRegisterManager->nodeRegisterMap["/r2/left_arm/joint0"].status["/r2/left_arm/joint0/StatReg1"].registerValue                   = 0x9053; // 1001 0000 0101 0011
    nodeRegisterManager->nodeRegisterMap["/r2/neck/joint0"].status["/r2/neck/joint0/StatReg1"].registerValue                           = 0x9053; // 1001 0000 0101 0011

    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::DRIVE, jointControlManagerMap["/r2/left_leg/joint0"]->getActualStates().controlMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::DRIVE, jointControlManagerMap["/r2/right_leg/gripper/joint0"]->getActualStates().controlMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::DRIVE, jointControlManagerMap["/r2/left_arm/joint0"]->getActualStates().controlMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::DRIVE, jointControlManagerMap["/r2/neck/joint0"]->getActualStates().controlMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::DRIVE, jointControlManagerMap["/r2/right_arm/wrist"]->getActualStates().controlMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::DRIVE, jointControlManagerMap["/r2/left_leg/joint0"]->getCommandStates().controlMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::DRIVE, jointControlManagerMap["/r2/right_leg/gripper/joint0"]->getCommandStates().controlMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::DRIVE, jointControlManagerMap["/r2/left_arm/joint0"]->getCommandStates().controlMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::DRIVE, jointControlManagerMap["/r2/neck/joint0"]->getCommandStates().controlMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::DRIVE, jointControlManagerMap["/r2/right_arm/wrist"]->getCommandStates().controlMode.state);

    // Fake coeffs loaded and joint faulted
    nodeRegisterManager->nodeRegisterMap["/r2/left_leg/joint0"].status["/r2/left_leg/joint0/StatReg2"].registerValue                   = 0x0180; // 0000 0001 1000 0000
    nodeRegisterManager->nodeRegisterMap["/r2/right_leg/gripper/joint0"].status["/r2/right_leg/gripper/joint0/StatReg2"].registerValue = 0x0180; // 0000 0001 1000 0000
    nodeRegisterManager->nodeRegisterMap["/r2/left_arm/joint0"].status["/r2/left_arm/joint0/StatReg2"].registerValue                   = 0x0180; // 0000 0001 1000 0000
    nodeRegisterManager->nodeRegisterMap["/r2/neck/joint0"].status["/r2/neck/joint0/StatReg2"].registerValue                           = 0x0180; // 0000 0001 1000 0000
    io.setLiveCoeff(StringUtilities::makeFullyQualifiedRoboDynElement("/r2/right_arm/wrist", "PitchLimit"), 1);

    EXPECT_FALSE(jointControlManagerMap["/r2/left_leg/joint0"]->verifyStates());
    EXPECT_FALSE(jointControlManagerMap["/r2/right_leg/gripper/joint0"]->verifyStates());
    EXPECT_FALSE(jointControlManagerMap["/r2/left_arm/joint0"]->verifyStates());
    EXPECT_FALSE(jointControlManagerMap["/r2/neck/joint0"]->verifyStates());
    EXPECT_FALSE(jointControlManagerMap["/r2/right_arm/wrist"]->verifyStates());

    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::FAULTED, jointControlManagerMap["/r2/left_leg/joint0"]->getActualStates().controlMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::FAULTED, jointControlManagerMap["/r2/right_leg/gripper/joint0"]->getActualStates().controlMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::FAULTED, jointControlManagerMap["/r2/left_arm/joint0"]->getActualStates().controlMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::FAULTED, jointControlManagerMap["/r2/neck/joint0"]->getActualStates().controlMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::FAULTED, jointControlManagerMap["/r2/right_arm/wrist"]->getActualStates().controlMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::DRIVE,   jointControlManagerMap["/r2/left_leg/joint0"]->getCommandStates().controlMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::DRIVE,   jointControlManagerMap["/r2/right_leg/gripper/joint0"]->getCommandStates().controlMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::DRIVE,   jointControlManagerMap["/r2/left_arm/joint0"]->getCommandStates().controlMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::DRIVE,   jointControlManagerMap["/r2/neck/joint0"]->getCommandStates().controlMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::DRIVE,   jointControlManagerMap["/r2/right_arm/wrist"]->getCommandStates().controlMode.state);
    commandMsg.controlMode.state = nasa_r2_common_msgs::JointControlMode::PARK;
    jointControlManagerMap["/r2/left_leg/joint0"]->setCommandStates(commandMsg);
    jointControlManagerMap["/r2/right_leg/gripper/joint0"]->setCommandStates(commandMsg);
    jointControlManagerMap["/r2/left_arm/joint0"]->setCommandStates(commandMsg);
    jointControlManagerMap["/r2/neck/joint0"]->setCommandStates(commandMsg);
    jointControlManagerMap["/r2/right_arm/wrist"]->setCommandStates(commandMsg);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::PARK, jointControlManagerMap["/r2/left_leg/joint0"]->getCommandStates().controlMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::PARK, jointControlManagerMap["/r2/right_leg/gripper/joint0"]->getCommandStates().controlMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::PARK, jointControlManagerMap["/r2/left_arm/joint0"]->getCommandStates().controlMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::PARK, jointControlManagerMap["/r2/neck/joint0"]->getCommandStates().controlMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::PARK, jointControlManagerMap["/r2/right_arm/wrist"]->getCommandStates().controlMode.state);
    EXPECT_EQ(1, nodeRegisterManager->getControlValue("/r2/left_leg/joint0",          "BootEnable"));
    EXPECT_EQ(1, nodeRegisterManager->getControlValue("/r2/left_leg/joint0",          "BridgeEnable"));
    EXPECT_EQ(1, nodeRegisterManager->getControlValue("/r2/left_leg/joint0",          "MotorEnable"));
    EXPECT_EQ(1, nodeRegisterManager->getControlValue("/r2/right_leg/gripper/joint0", "BootEnable"));
    EXPECT_EQ(1, nodeRegisterManager->getControlValue("/r2/right_leg/gripper/joint0", "BridgeEnable"));
    EXPECT_EQ(1, nodeRegisterManager->getControlValue("/r2/right_leg/gripper/joint0", "MotorEnable"));
    EXPECT_EQ(1, nodeRegisterManager->getControlValue("/r2/left_arm/joint0",          "BootEnable"));
    EXPECT_EQ(1, nodeRegisterManager->getControlValue("/r2/left_arm/joint0",          "BridgeEnable"));
    EXPECT_EQ(1, nodeRegisterManager->getControlValue("/r2/left_arm/joint0",          "MotorEnable"));
    EXPECT_EQ(0, nodeRegisterManager->getControlValue("/r2/left_arm/joint0",          "BrakeRelease"));
    EXPECT_EQ(1, nodeRegisterManager->getControlValue("/r2/neck/joint0",              "BootEnable"));
    EXPECT_EQ(1, nodeRegisterManager->getControlValue("/r2/neck/joint0",              "BridgeEnable"));
    EXPECT_EQ(1, nodeRegisterManager->getControlValue("/r2/neck/joint0",              "MotorEnable"));
    EXPECT_EQ(0, nodeRegisterManager->getControlValue("/r2/neck/joint0",              "BrakeRelease"));

    commandMsg.clearFaultMode.state = nasa_r2_common_msgs::JointControlClearFaultMode::ENABLE;
    jointControlManagerMap["/r2/left_leg/joint0"]->setCommandStates(commandMsg);
    jointControlManagerMap["/r2/right_leg/gripper/joint0"]->setCommandStates(commandMsg);
    jointControlManagerMap["/r2/left_arm/joint0"]->setCommandStates(commandMsg);
    jointControlManagerMap["/r2/neck/joint0"]->setCommandStates(commandMsg);
    jointControlManagerMap["/r2/right_arm/wrist"]->setCommandStates(commandMsg);

    EXPECT_EQ(1, nodeRegisterManager->getControlValue("/r2/left_leg/joint0",          "ClearFault"));
    EXPECT_EQ(1, nodeRegisterManager->getControlValue("/r2/right_leg/gripper/joint0", "ClearFault"));
    EXPECT_EQ(1, nodeRegisterManager->getControlValue("/r2/left_arm/joint0",          "ClearFault"));
    EXPECT_EQ(1, nodeRegisterManager->getControlValue("/r2/neck/joint0",              "ClearFault"));

    commandMsg.clearFaultMode.state = nasa_r2_common_msgs::JointControlClearFaultMode::DISABLE;
    jointControlManagerMap["/r2/left_leg/joint0"]->setCommandStates(commandMsg);
    jointControlManagerMap["/r2/right_leg/gripper/joint0"]->setCommandStates(commandMsg);
    jointControlManagerMap["/r2/left_arm/joint0"]->setCommandStates(commandMsg);
    jointControlManagerMap["/r2/neck/joint0"]->setCommandStates(commandMsg);
    jointControlManagerMap["/r2/right_arm/wrist"]->setCommandStates(commandMsg);
    nodeRegisterManager->nodeRegisterMap["/r2/left_leg/joint0"].status["/r2/left_leg/joint0/StatReg1"].registerValue                   = 0x9011; // 1001 0000 0001 0001
    nodeRegisterManager->nodeRegisterMap["/r2/right_leg/gripper/joint0"].status["/r2/right_leg/gripper/joint0/StatReg1"].registerValue = 0x9011; // 1001 0000 0001 0001
    nodeRegisterManager->nodeRegisterMap["/r2/left_arm/joint0"].status["/r2/left_arm/joint0/StatReg1"].registerValue                   = 0x9011; // 1001 0000 0001 0001
    nodeRegisterManager->nodeRegisterMap["/r2/neck/joint0"].status["/r2/neck/joint0/StatReg1"].registerValue                           = 0x9011; // 1001 0000 0001 0001
    nodeRegisterManager->nodeRegisterMap["/r2/left_leg/joint0"].status["/r2/left_leg/joint0/StatReg2"].registerValue                   = 0x0000; // 0000 0000 0000 0000
    nodeRegisterManager->nodeRegisterMap["/r2/right_leg/gripper/joint0"].status["/r2/right_leg/gripper/joint0/StatReg2"].registerValue = 0x0000; // 0000 0000 0000 0000
    nodeRegisterManager->nodeRegisterMap["/r2/left_arm/joint0"].status["/r2/left_arm/joint0/StatReg2"].registerValue                   = 0x0000; // 0000 0000 0000 0000
    nodeRegisterManager->nodeRegisterMap["/r2/neck/joint0"].status["/r2/neck/joint0/StatReg2"].registerValue                           = 0x0000; // 0000 0000 0000 0000
    io.setLiveCoeff(StringUtilities::makeFullyQualifiedRoboDynElement("/r2/right_arm/wrist", "PitchLimit"), 0);

    EXPECT_EQ(0, nodeRegisterManager->getControlValue("/r2/left_leg/joint0",          "ClearFault"));
    EXPECT_EQ(0, nodeRegisterManager->getControlValue("/r2/right_leg/gripper/joint0", "ClearFault"));
    EXPECT_EQ(0, nodeRegisterManager->getControlValue("/r2/left_arm/joint0",          "ClearFault"));
    EXPECT_EQ(0, nodeRegisterManager->getControlValue("/r2/neck/joint0",              "ClearFault"));
    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::PARK, jointControlManagerMap["/r2/left_leg/joint0"]->getActualStates().controlMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::PARK, jointControlManagerMap["/r2/right_leg/gripper/joint0"]->getActualStates().controlMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::PARK, jointControlManagerMap["/r2/left_arm/joint0"]->getActualStates().controlMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::PARK, jointControlManagerMap["/r2/neck/joint0"]->getActualStates().controlMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::PARK, jointControlManagerMap["/r2/right_arm/wrist"]->getActualStates().controlMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::PARK, jointControlManagerMap["/r2/left_leg/joint0"]->getCommandStates().controlMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::PARK, jointControlManagerMap["/r2/right_leg/gripper/joint0"]->getCommandStates().controlMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::PARK, jointControlManagerMap["/r2/left_arm/joint0"]->getCommandStates().controlMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::PARK, jointControlManagerMap["/r2/neck/joint0"]->getCommandStates().controlMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::PARK, jointControlManagerMap["/r2/right_arm/wrist"]->getCommandStates().controlMode.state);
}

TEST_F(JointControlManagerTest, InitialCommandModeStates)
{
    EXPECT_EQ(nasa_r2_common_msgs::JointControlCommandMode::MOTCOM, jointControlManagerMap["/r2/left_leg/joint0"]->getActualStates().commandMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlCommandMode::MOTCOM, jointControlManagerMap["/r2/right_leg/gripper/joint0"]->getActualStates().commandMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlCommandMode::MOTCOM, jointControlManagerMap["/r2/left_arm/joint0"]->getActualStates().commandMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlCommandMode::MOTCOM, jointControlManagerMap["/r2/neck/joint0"]->getActualStates().commandMode.state);

    EXPECT_EQ(nasa_r2_common_msgs::JointControlCommandMode::MOTCOM, jointControlManagerMap["/r2/left_leg/joint0"]->getCommandStates().commandMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlCommandMode::MOTCOM, jointControlManagerMap["/r2/right_leg/gripper/joint0"]->getCommandStates().commandMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlCommandMode::MOTCOM, jointControlManagerMap["/r2/left_arm/joint0"]->getCommandStates().commandMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlCommandMode::MOTCOM, jointControlManagerMap["/r2/neck/joint0"]->getCommandStates().commandMode.state);

    EXPECT_EQ(0, nodeRegisterManager->getControlValue("/r2/left_leg/joint0",          "MotComSource"));
    EXPECT_EQ(0, nodeRegisterManager->getControlValue("/r2/right_leg/gripper/joint0", "MotComSource"));
    EXPECT_EQ(0, nodeRegisterManager->getControlValue("/r2/left_arm/joint0",          "MotComSource"));
    EXPECT_EQ(0, nodeRegisterManager->getControlValue("/r2/neck/joint0",              "MotComSource"));
}

TEST_F(JointControlManagerTest, MultiLoopStepThenMotcomThenBack)
{
    // Fake coeffs loaded and MultiLoop
    nodeRegisterManager->nodeRegisterMap["/r2/left_leg/joint0"].status["/r2/left_leg/joint0/StatReg2"].registerValue                   = 0x0103; // 0000 0001 0000 0011
    nodeRegisterManager->nodeRegisterMap["/r2/right_leg/gripper/joint0"].status["/r2/right_leg/gripper/joint0/StatReg2"].registerValue = 0x0103; // 0000 0001 0000 0011
    nodeRegisterManager->nodeRegisterMap["/r2/left_arm/joint0"].status["/r2/left_arm/joint0/StatReg2"].registerValue                   = 0x0103; // 0000 0001 0000 0011
    nodeRegisterManager->nodeRegisterMap["/r2/neck/joint0"].status["/r2/neck/joint0/StatReg2"].registerValue                           = 0x0103; // 0000 0001 0000 0011

    // MultiLoopStep
    commandMsg.commandMode.state = nasa_r2_common_msgs::JointControlCommandMode::MULTILOOPSTEP;
    jointControlManagerMap["/r2/left_leg/joint0"]->setCommandStates(commandMsg);
    jointControlManagerMap["/r2/right_leg/gripper/joint0"]->setCommandStates(commandMsg);
    jointControlManagerMap["/r2/left_arm/joint0"]->setCommandStates(commandMsg);
    jointControlManagerMap["/r2/neck/joint0"]->setCommandStates(commandMsg);

    // Fake MotComSource == 1
    nodeRegisterManager->nodeRegisterMap["/r2/left_leg/joint0"].status["/r2/left_leg/joint0/StatReg1"].registerValue                   = 0x9040; // 1001 0000 0100 0000
    nodeRegisterManager->nodeRegisterMap["/r2/right_leg/gripper/joint0"].status["/r2/right_leg/gripper/joint0/StatReg1"].registerValue = 0x9040; // 1001 0000 0100 0000
    nodeRegisterManager->nodeRegisterMap["/r2/left_arm/joint0"].status["/r2/left_arm/joint0/StatReg1"].registerValue                   = 0x9040; // 1001 0000 0100 0000
    nodeRegisterManager->nodeRegisterMap["/r2/neck/joint0"].status["/r2/neck/joint0/StatReg1"].registerValue                           = 0x9040; // 1001 0000 0100 0000

    EXPECT_EQ(nasa_r2_common_msgs::JointControlCommandMode::MULTILOOPSTEP, jointControlManagerMap["/r2/left_leg/joint0"]->getActualStates().commandMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlCommandMode::MULTILOOPSTEP, jointControlManagerMap["/r2/right_leg/gripper/joint0"]->getActualStates().commandMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlCommandMode::MULTILOOPSTEP, jointControlManagerMap["/r2/left_arm/joint0"]->getActualStates().commandMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlCommandMode::MULTILOOPSTEP, jointControlManagerMap["/r2/neck/joint0"]->getActualStates().commandMode.state);

    EXPECT_EQ(nasa_r2_common_msgs::JointControlCommandMode::MULTILOOPSTEP, jointControlManagerMap["/r2/left_leg/joint0"]->getCommandStates().commandMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlCommandMode::MULTILOOPSTEP, jointControlManagerMap["/r2/right_leg/gripper/joint0"]->getCommandStates().commandMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlCommandMode::MULTILOOPSTEP, jointControlManagerMap["/r2/left_arm/joint0"]->getCommandStates().commandMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlCommandMode::MULTILOOPSTEP, jointControlManagerMap["/r2/neck/joint0"]->getCommandStates().commandMode.state);

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

    // MotCom
    commandMsg.commandMode.state = nasa_r2_common_msgs::JointControlCommandMode::MOTCOM;
    jointControlManagerMap["/r2/left_leg/joint0"]->setCommandStates(commandMsg);
    jointControlManagerMap["/r2/right_leg/gripper/joint0"]->setCommandStates(commandMsg);
    jointControlManagerMap["/r2/left_arm/joint0"]->setCommandStates(commandMsg);
    jointControlManagerMap["/r2/neck/joint0"]->setCommandStates(commandMsg);

    // UnFake MotComSource == 1
    nodeRegisterManager->nodeRegisterMap["/r2/left_leg/joint0"].status["/r2/left_leg/joint0/StatReg1"].registerValue                   = 0x9000; // 1001 0000 0000 0000
    nodeRegisterManager->nodeRegisterMap["/r2/right_leg/gripper/joint0"].status["/r2/right_leg/gripper/joint0/StatReg1"].registerValue = 0x9000; // 1001 0000 0000 0000
    nodeRegisterManager->nodeRegisterMap["/r2/left_arm/joint0"].status["/r2/left_arm/joint0/StatReg1"].registerValue                   = 0x9000; // 1001 0000 0000 0000
    nodeRegisterManager->nodeRegisterMap["/r2/neck/joint0"].status["/r2/neck/joint0/StatReg1"].registerValue                           = 0x9000; // 1001 0000 0000 0000

    EXPECT_EQ(nasa_r2_common_msgs::JointControlCommandMode::MOTCOM, jointControlManagerMap["/r2/left_leg/joint0"]->getActualStates().commandMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlCommandMode::MOTCOM, jointControlManagerMap["/r2/right_leg/gripper/joint0"]->getActualStates().commandMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlCommandMode::MOTCOM, jointControlManagerMap["/r2/left_arm/joint0"]->getActualStates().commandMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlCommandMode::MOTCOM, jointControlManagerMap["/r2/neck/joint0"]->getActualStates().commandMode.state);

    EXPECT_EQ(nasa_r2_common_msgs::JointControlCommandMode::MOTCOM, jointControlManagerMap["/r2/left_leg/joint0"]->getCommandStates().commandMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlCommandMode::MOTCOM, jointControlManagerMap["/r2/right_leg/gripper/joint0"]->getCommandStates().commandMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlCommandMode::MOTCOM, jointControlManagerMap["/r2/left_arm/joint0"]->getCommandStates().commandMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlCommandMode::MOTCOM, jointControlManagerMap["/r2/neck/joint0"]->getCommandStates().commandMode.state);

    EXPECT_EQ(0, nodeRegisterManager->getControlValue("/r2/left_leg/joint0",          "MotComSource"));
    EXPECT_EQ(0, nodeRegisterManager->getControlValue("/r2/right_leg/gripper/joint0", "MotComSource"));
    EXPECT_EQ(0, nodeRegisterManager->getControlValue("/r2/left_arm/joint0",          "MotComSource"));
    EXPECT_EQ(0, nodeRegisterManager->getControlValue("/r2/neck/joint0",              "MotComSource"));

    // MultiLoopStep
    commandMsg.commandMode.state = nasa_r2_common_msgs::JointControlCommandMode::MULTILOOPSTEP;
    jointControlManagerMap["/r2/left_leg/joint0"]->setCommandStates(commandMsg);
    jointControlManagerMap["/r2/right_leg/gripper/joint0"]->setCommandStates(commandMsg);
    jointControlManagerMap["/r2/left_arm/joint0"]->setCommandStates(commandMsg);
    jointControlManagerMap["/r2/neck/joint0"]->setCommandStates(commandMsg);

    // Fake MotComSource == 1
    nodeRegisterManager->nodeRegisterMap["/r2/left_leg/joint0"].status["/r2/left_leg/joint0/StatReg1"].registerValue                   = 0x9040; // 1001 0000 0100 0000
    nodeRegisterManager->nodeRegisterMap["/r2/right_leg/gripper/joint0"].status["/r2/right_leg/gripper/joint0/StatReg1"].registerValue = 0x9040; // 1001 0000 0100 0000
    nodeRegisterManager->nodeRegisterMap["/r2/left_arm/joint0"].status["/r2/left_arm/joint0/StatReg1"].registerValue                   = 0x9040; // 1001 0000 0100 0000
    nodeRegisterManager->nodeRegisterMap["/r2/neck/joint0"].status["/r2/neck/joint0/StatReg1"].registerValue                           = 0x9040; // 1001 0000 0100 0000

    EXPECT_EQ(nasa_r2_common_msgs::JointControlCommandMode::MULTILOOPSTEP, jointControlManagerMap["/r2/left_leg/joint0"]->getActualStates().commandMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlCommandMode::MULTILOOPSTEP, jointControlManagerMap["/r2/right_leg/gripper/joint0"]->getActualStates().commandMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlCommandMode::MULTILOOPSTEP, jointControlManagerMap["/r2/left_arm/joint0"]->getActualStates().commandMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlCommandMode::MULTILOOPSTEP, jointControlManagerMap["/r2/neck/joint0"]->getActualStates().commandMode.state);

    EXPECT_EQ(nasa_r2_common_msgs::JointControlCommandMode::MULTILOOPSTEP, jointControlManagerMap["/r2/left_leg/joint0"]->getCommandStates().commandMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlCommandMode::MULTILOOPSTEP, jointControlManagerMap["/r2/right_leg/gripper/joint0"]->getCommandStates().commandMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlCommandMode::MULTILOOPSTEP, jointControlManagerMap["/r2/left_arm/joint0"]->getCommandStates().commandMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlCommandMode::MULTILOOPSTEP, jointControlManagerMap["/r2/neck/joint0"]->getCommandStates().commandMode.state);

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

TEST_F(JointControlManagerTest, StallMode)
{
    // Fake coeffs loaded and StallMode
    nodeRegisterManager->nodeRegisterMap["/r2/left_leg/joint0"].status["/r2/left_leg/joint0/StatReg2"].registerValue                   = 0x0101; // 0000 0001 0000 0001
    nodeRegisterManager->nodeRegisterMap["/r2/right_leg/gripper/joint0"].status["/r2/right_leg/gripper/joint0/StatReg2"].registerValue = 0x0101; // 0000 0001 0000 0001
    nodeRegisterManager->nodeRegisterMap["/r2/left_arm/joint0"].status["/r2/left_arm/joint0/StatReg2"].registerValue                   = 0x0101; // 0000 0001 0000 0001
    nodeRegisterManager->nodeRegisterMap["/r2/neck/joint0"].status["/r2/neck/joint0/StatReg2"].registerValue                           = 0x0101; // 0000 0001 0000 0001

    // StallMode
    commandMsg.commandMode.state = nasa_r2_common_msgs::JointControlCommandMode::STALLMODE;
    jointControlManagerMap["/r2/left_leg/joint0"]->setCommandStates(commandMsg);
    jointControlManagerMap["/r2/right_leg/gripper/joint0"]->setCommandStates(commandMsg);
    jointControlManagerMap["/r2/left_arm/joint0"]->setCommandStates(commandMsg);
    jointControlManagerMap["/r2/neck/joint0"]->setCommandStates(commandMsg);

    // Fake MotComSource == 1
    nodeRegisterManager->nodeRegisterMap["/r2/left_leg/joint0"].status["/r2/left_leg/joint0/StatReg1"].registerValue                   = 0x9040; // 1001 0000 0100 0000
    nodeRegisterManager->nodeRegisterMap["/r2/right_leg/gripper/joint0"].status["/r2/right_leg/gripper/joint0/StatReg1"].registerValue = 0x9040; // 1001 0000 0100 0000
    nodeRegisterManager->nodeRegisterMap["/r2/left_arm/joint0"].status["/r2/left_arm/joint0/StatReg1"].registerValue                   = 0x9040; // 1001 0000 0100 0000
    nodeRegisterManager->nodeRegisterMap["/r2/neck/joint0"].status["/r2/neck/joint0/StatReg1"].registerValue                           = 0x9040; // 1001 0000 0100 0000

    EXPECT_EQ(nasa_r2_common_msgs::JointControlCommandMode::STALLMODE, jointControlManagerMap["/r2/left_leg/joint0"]->getActualStates().commandMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlCommandMode::STALLMODE, jointControlManagerMap["/r2/right_leg/gripper/joint0"]->getActualStates().commandMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlCommandMode::STALLMODE, jointControlManagerMap["/r2/left_arm/joint0"]->getActualStates().commandMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlCommandMode::STALLMODE, jointControlManagerMap["/r2/neck/joint0"]->getActualStates().commandMode.state);

    EXPECT_EQ(nasa_r2_common_msgs::JointControlCommandMode::STALLMODE, jointControlManagerMap["/r2/left_leg/joint0"]->getCommandStates().commandMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlCommandMode::STALLMODE, jointControlManagerMap["/r2/right_leg/gripper/joint0"]->getCommandStates().commandMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlCommandMode::STALLMODE, jointControlManagerMap["/r2/left_arm/joint0"]->getCommandStates().commandMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlCommandMode::STALLMODE, jointControlManagerMap["/r2/neck/joint0"]->getCommandStates().commandMode.state);

    EXPECT_EQ(1, nodeRegisterManager->getControlValue("/r2/left_leg/joint0", "MotComSource"));
    EXPECT_EQ(1, nodeRegisterManager->getControlValue("/r2/left_leg/joint0", "ControlMode"));

    EXPECT_EQ(1, nodeRegisterManager->getControlValue("/r2/right_leg/gripper/joint0", "MotComSource"));
    EXPECT_EQ(1, nodeRegisterManager->getControlValue("/r2/right_leg/gripper/joint0", "ControlMode"));

    EXPECT_EQ(1, nodeRegisterManager->getControlValue("/r2/left_arm/joint0", "MotComSource"));
    EXPECT_EQ(1, nodeRegisterManager->getControlValue("/r2/left_arm/joint0", "ControlMode"));

    EXPECT_EQ(1, nodeRegisterManager->getControlValue("/r2/neck/joint0", "MotComSource"));
    EXPECT_EQ(1, nodeRegisterManager->getControlValue("/r2/neck/joint0", "ControlMode"));
}

TEST_F(JointControlManagerTest, MultiLoopSmooth)
{
    // Fake coeffs loaded and MultiLoop
    nodeRegisterManager->nodeRegisterMap["/r2/left_leg/joint0"].status["/r2/left_leg/joint0/StatReg2"].registerValue                   = 0x0103; // 0000 0001 0000 0011
    nodeRegisterManager->nodeRegisterMap["/r2/right_leg/gripper/joint0"].status["/r2/right_leg/gripper/joint0/StatReg2"].registerValue = 0x0103; // 0000 0001 0000 0011
    nodeRegisterManager->nodeRegisterMap["/r2/left_arm/joint0"].status["/r2/left_arm/joint0/StatReg2"].registerValue                   = 0x0103; // 0000 0001 0000 0011
    nodeRegisterManager->nodeRegisterMap["/r2/neck/joint0"].status["/r2/neck/joint0/StatReg2"].registerValue                           = 0x0103; // 0000 0001 0000 0011

    // MultiLoopSmooth
    commandMsg.commandMode.state = nasa_r2_common_msgs::JointControlCommandMode::MULTILOOPSMOOTH;
    jointControlManagerMap["/r2/left_leg/joint0"]->setCommandStates(commandMsg);
    jointControlManagerMap["/r2/right_leg/gripper/joint0"]->setCommandStates(commandMsg);
    jointControlManagerMap["/r2/left_arm/joint0"]->setCommandStates(commandMsg);
    jointControlManagerMap["/r2/neck/joint0"]->setCommandStates(commandMsg);

    // Fake MotComSource == 1
    nodeRegisterManager->nodeRegisterMap["/r2/left_leg/joint0"].status["/r2/left_leg/joint0/StatReg1"].registerValue                   = 0x9040; // 1001 0000 0100 0000
    nodeRegisterManager->nodeRegisterMap["/r2/right_leg/gripper/joint0"].status["/r2/right_leg/gripper/joint0/StatReg1"].registerValue = 0x9040; // 1001 0000 0100 0000
    nodeRegisterManager->nodeRegisterMap["/r2/left_arm/joint0"].status["/r2/left_arm/joint0/StatReg1"].registerValue                   = 0x9040; // 1001 0000 0100 0000
    nodeRegisterManager->nodeRegisterMap["/r2/neck/joint0"].status["/r2/neck/joint0/StatReg1"].registerValue                           = 0x9040; // 1001 0000 0100 0000

    // Fake PosComVelocity == 1
    nodeRegisterManager->nodeRegisterMap["/r2/left_leg/joint0"].status["/r2/left_leg/joint0/StatReg3"].registerValue = 0x0100; // 0000 0001 0000 0000
    nodeRegisterManager->nodeRegisterMap["/r2/left_arm/joint0"].status["/r2/left_arm/joint0/StatReg3"].registerValue = 0x0100; // 0000 0001 0000 0000
    nodeRegisterManager->nodeRegisterMap["/r2/neck/joint0"].status["/r2/neck/joint0/StatReg3"].registerValue         = 0x0100; // 0000 0001 0000 0000

    EXPECT_EQ(nasa_r2_common_msgs::JointControlCommandMode::MULTILOOPSMOOTH, jointControlManagerMap["/r2/left_leg/joint0"]->getActualStates().commandMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlCommandMode::MULTILOOPSTEP,   jointControlManagerMap["/r2/right_leg/gripper/joint0"]->getActualStates().commandMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlCommandMode::MULTILOOPSMOOTH, jointControlManagerMap["/r2/left_arm/joint0"]->getActualStates().commandMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlCommandMode::MULTILOOPSMOOTH, jointControlManagerMap["/r2/neck/joint0"]->getActualStates().commandMode.state);

    EXPECT_EQ(nasa_r2_common_msgs::JointControlCommandMode::MULTILOOPSMOOTH, jointControlManagerMap["/r2/left_leg/joint0"]->getCommandStates().commandMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlCommandMode::MULTILOOPSTEP,   jointControlManagerMap["/r2/right_leg/gripper/joint0"]->getCommandStates().commandMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlCommandMode::MULTILOOPSMOOTH, jointControlManagerMap["/r2/left_arm/joint0"]->getCommandStates().commandMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlCommandMode::MULTILOOPSMOOTH, jointControlManagerMap["/r2/neck/joint0"]->getCommandStates().commandMode.state);

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

TEST_F(JointControlManagerTest, InitialCalibrationModeStates)
{
    EXPECT_EQ(nasa_r2_common_msgs::JointControlCalibrationMode::DISABLE, jointControlManagerMap["/r2/left_leg/joint0"]->getActualStates().calibrationMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlCalibrationMode::DISABLE, jointControlManagerMap["/r2/right_leg/gripper/joint0"]->getActualStates().calibrationMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlCalibrationMode::DISABLE, jointControlManagerMap["/r2/left_arm/joint0"]->getActualStates().calibrationMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlCalibrationMode::DISABLE, jointControlManagerMap["/r2/neck/joint0"]->getActualStates().calibrationMode.state);

    EXPECT_EQ(nasa_r2_common_msgs::JointControlCalibrationMode::DISABLE, jointControlManagerMap["/r2/left_leg/joint0"]->getCommandStates().calibrationMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlCalibrationMode::DISABLE, jointControlManagerMap["/r2/right_leg/gripper/joint0"]->getCommandStates().calibrationMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlCalibrationMode::DISABLE, jointControlManagerMap["/r2/left_arm/joint0"]->getCommandStates().calibrationMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlCalibrationMode::DISABLE, jointControlManagerMap["/r2/neck/joint0"]->getCommandStates().calibrationMode.state);

    EXPECT_EQ(0, nodeRegisterManager->getControlValue("/r2/left_leg/joint0",          "CalibrationMode"));
    EXPECT_EQ(0, nodeRegisterManager->getControlValue("/r2/right_leg/gripper/joint0", "CalibrationMode"));
    EXPECT_EQ(0, nodeRegisterManager->getControlValue("/r2/left_arm/joint0",          "CalibrationMode"));
    EXPECT_EQ(0, nodeRegisterManager->getControlValue("/r2/neck/joint0",              "CalibrationMode"));
}

TEST_F(JointControlManagerTest, EnableCalibrationMode)
{
    commandMsg.calibrationMode.state = nasa_r2_common_msgs::JointControlCalibrationMode::ENABLE;
    jointControlManagerMap["/r2/left_leg/joint0"]->setCommandStates(commandMsg);
    jointControlManagerMap["/r2/right_leg/gripper/joint0"]->setCommandStates(commandMsg);
    jointControlManagerMap["/r2/left_arm/joint0"]->setCommandStates(commandMsg);
    jointControlManagerMap["/r2/neck/joint0"]->setCommandStates(commandMsg);

    nodeRegisterManager->nodeRegisterMap["/r2/left_leg/joint0"].status["/r2/left_leg/joint0/StatReg2"].registerValue                   = 0x8000; // 1000 0000 0000 0000
    nodeRegisterManager->nodeRegisterMap["/r2/right_leg/gripper/joint0"].status["/r2/right_leg/gripper/joint0/StatReg2"].registerValue = 0x8000; // 1000 0000 0000 0000
    nodeRegisterManager->nodeRegisterMap["/r2/left_arm/joint0"].status["/r2/left_arm/joint0/StatReg2"].registerValue                   = 0x8000; // 1000 0000 0000 0000
    nodeRegisterManager->nodeRegisterMap["/r2/neck/joint0"].status["/r2/neck/joint0/StatReg2"].registerValue                           = 0x8000; // 1000 0000 0000 0000

    EXPECT_EQ(nasa_r2_common_msgs::JointControlCalibrationMode::ENABLE, jointControlManagerMap["/r2/left_leg/joint0"]->getActualStates().calibrationMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlCalibrationMode::ENABLE, jointControlManagerMap["/r2/right_leg/gripper/joint0"]->getActualStates().calibrationMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlCalibrationMode::ENABLE, jointControlManagerMap["/r2/left_arm/joint0"]->getActualStates().calibrationMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlCalibrationMode::ENABLE, jointControlManagerMap["/r2/neck/joint0"]->getActualStates().calibrationMode.state);

    EXPECT_EQ(nasa_r2_common_msgs::JointControlCalibrationMode::ENABLE, jointControlManagerMap["/r2/left_leg/joint0"]->getCommandStates().calibrationMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlCalibrationMode::ENABLE, jointControlManagerMap["/r2/right_leg/gripper/joint0"]->getCommandStates().calibrationMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlCalibrationMode::ENABLE, jointControlManagerMap["/r2/left_arm/joint0"]->getCommandStates().calibrationMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlCalibrationMode::ENABLE, jointControlManagerMap["/r2/neck/joint0"]->getCommandStates().calibrationMode.state);

    EXPECT_EQ(1, nodeRegisterManager->getControlValue("/r2/left_leg/joint0",          "CalibrationMode"));
    EXPECT_EQ(1, nodeRegisterManager->getControlValue("/r2/right_leg/gripper/joint0", "CalibrationMode"));
    EXPECT_EQ(1, nodeRegisterManager->getControlValue("/r2/left_arm/joint0",          "CalibrationMode"));
    EXPECT_EQ(1, nodeRegisterManager->getControlValue("/r2/neck/joint0",              "CalibrationMode"));
}

TEST_F(JointControlManagerTest, InitialClearFaultModeStates)
{
    EXPECT_EQ(nasa_r2_common_msgs::JointControlClearFaultMode::DISABLE, jointControlManagerMap["/r2/left_leg/joint0"]->getActualStates().clearFaultMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlClearFaultMode::DISABLE, jointControlManagerMap["/r2/right_leg/gripper/joint0"]->getActualStates().clearFaultMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlClearFaultMode::DISABLE, jointControlManagerMap["/r2/left_arm/joint0"]->getActualStates().clearFaultMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlClearFaultMode::DISABLE, jointControlManagerMap["/r2/neck/joint0"]->getActualStates().clearFaultMode.state);

    EXPECT_EQ(nasa_r2_common_msgs::JointControlClearFaultMode::DISABLE, jointControlManagerMap["/r2/left_leg/joint0"]->getCommandStates().clearFaultMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlClearFaultMode::DISABLE, jointControlManagerMap["/r2/right_leg/gripper/joint0"]->getCommandStates().clearFaultMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlClearFaultMode::DISABLE, jointControlManagerMap["/r2/left_arm/joint0"]->getCommandStates().clearFaultMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlClearFaultMode::DISABLE, jointControlManagerMap["/r2/neck/joint0"]->getCommandStates().clearFaultMode.state);

    EXPECT_EQ(0, nodeRegisterManager->getControlValue("/r2/left_leg/joint0",          "ClearFault"));
    EXPECT_EQ(0, nodeRegisterManager->getControlValue("/r2/right_leg/gripper/joint0", "ClearFault"));
    EXPECT_EQ(0, nodeRegisterManager->getControlValue("/r2/left_arm/joint0",          "ClearFault"));
    EXPECT_EQ(0, nodeRegisterManager->getControlValue("/r2/neck/joint0",              "ClearFault"));
}

TEST_F(JointControlManagerTest, EnableClearFaultMode)
{
    commandMsg.clearFaultMode.state = nasa_r2_common_msgs::JointControlClearFaultMode::ENABLE;
    jointControlManagerMap["/r2/left_leg/joint0"]->setCommandStates(commandMsg);
    jointControlManagerMap["/r2/right_leg/gripper/joint0"]->setCommandStates(commandMsg);
    jointControlManagerMap["/r2/left_arm/joint0"]->setCommandStates(commandMsg);
    jointControlManagerMap["/r2/neck/joint0"]->setCommandStates(commandMsg);

    nodeRegisterManager->nodeRegisterMap["/r2/left_leg/joint0"].status["/r2/left_leg/joint0/StatReg2"].registerValue                   = 0x0040; // 0000 0000 0100 0000
    nodeRegisterManager->nodeRegisterMap["/r2/right_leg/gripper/joint0"].status["/r2/right_leg/gripper/joint0/StatReg2"].registerValue = 0x0040; // 0000 0000 0100 0000
    nodeRegisterManager->nodeRegisterMap["/r2/left_arm/joint0"].status["/r2/left_arm/joint0/StatReg2"].registerValue                   = 0x0040; // 0000 0000 0100 0000
    nodeRegisterManager->nodeRegisterMap["/r2/neck/joint0"].status["/r2/neck/joint0/StatReg2"].registerValue                           = 0x0040; // 0000 0000 0100 0000

    EXPECT_EQ(nasa_r2_common_msgs::JointControlClearFaultMode::ENABLE, jointControlManagerMap["/r2/left_leg/joint0"]->getActualStates().clearFaultMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlClearFaultMode::ENABLE, jointControlManagerMap["/r2/right_leg/gripper/joint0"]->getActualStates().clearFaultMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlClearFaultMode::ENABLE, jointControlManagerMap["/r2/left_arm/joint0"]->getActualStates().clearFaultMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlClearFaultMode::ENABLE, jointControlManagerMap["/r2/neck/joint0"]->getActualStates().clearFaultMode.state);

    EXPECT_EQ(nasa_r2_common_msgs::JointControlClearFaultMode::ENABLE, jointControlManagerMap["/r2/left_leg/joint0"]->getCommandStates().clearFaultMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlClearFaultMode::ENABLE, jointControlManagerMap["/r2/right_leg/gripper/joint0"]->getCommandStates().clearFaultMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlClearFaultMode::ENABLE, jointControlManagerMap["/r2/left_arm/joint0"]->getCommandStates().clearFaultMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlClearFaultMode::ENABLE, jointControlManagerMap["/r2/neck/joint0"]->getCommandStates().clearFaultMode.state);

    EXPECT_EQ(1, nodeRegisterManager->getControlValue("/r2/left_leg/joint0",          "ClearFault"));
    EXPECT_EQ(1, nodeRegisterManager->getControlValue("/r2/right_leg/gripper/joint0", "ClearFault"));
    EXPECT_EQ(1, nodeRegisterManager->getControlValue("/r2/left_arm/joint0",          "ClearFault"));
    EXPECT_EQ(1, nodeRegisterManager->getControlValue("/r2/neck/joint0",              "ClearFault"));
}

TEST_F(JointControlManagerTest, VerifyControlModeState)
{
    EXPECT_TRUE(jointControlManagerMap["/r2/left_leg/joint0"]->verifyStates());
    EXPECT_TRUE(jointControlManagerMap["/r2/right_leg/gripper/joint0"]->verifyStates());
    EXPECT_TRUE(jointControlManagerMap["/r2/left_arm/joint0"]->verifyStates());
    EXPECT_TRUE(jointControlManagerMap["/r2/neck/joint0"]->verifyStates());

    commandMsg.controlMode.state = nasa_r2_common_msgs::JointControlMode::OFF;
    jointControlManagerMap["/r2/left_leg/joint0"]->setCommandStates(commandMsg);
    jointControlManagerMap["/r2/right_leg/gripper/joint0"]->setCommandStates(commandMsg);
    jointControlManagerMap["/r2/left_arm/joint0"]->setCommandStates(commandMsg);
    jointControlManagerMap["/r2/neck/joint0"]->setCommandStates(commandMsg);
    commandMsg.controlMode.state = nasa_r2_common_msgs::JointControlMode::PARK;
    jointControlManagerMap["/r2/left_leg/joint0"]->setCommandStates(commandMsg);
    jointControlManagerMap["/r2/right_leg/gripper/joint0"]->setCommandStates(commandMsg);
    jointControlManagerMap["/r2/left_arm/joint0"]->setCommandStates(commandMsg);
    jointControlManagerMap["/r2/neck/joint0"]->setCommandStates(commandMsg);

    usleep(timeLimit*1000000);
    EXPECT_FALSE(jointControlManagerMap["/r2/left_leg/joint0"]->verifyStates());
    EXPECT_FALSE(jointControlManagerMap["/r2/right_leg/gripper/joint0"]->verifyStates());
    EXPECT_FALSE(jointControlManagerMap["/r2/left_arm/joint0"]->verifyStates());
    EXPECT_FALSE(jointControlManagerMap["/r2/neck/joint0"]->verifyStates());

    EXPECT_TRUE(jointControlManagerMap["/r2/left_leg/joint0"]->verifyStates());
    EXPECT_TRUE(jointControlManagerMap["/r2/right_leg/gripper/joint0"]->verifyStates());
    EXPECT_TRUE(jointControlManagerMap["/r2/left_arm/joint0"]->verifyStates());
    EXPECT_TRUE(jointControlManagerMap["/r2/neck/joint0"]->verifyStates());

    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::OFF,  jointControlManagerMap["/r2/left_leg/joint0"]->getActualStates().controlMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::OFF,  jointControlManagerMap["/r2/right_leg/gripper/joint0"]->getActualStates().controlMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::OFF,  jointControlManagerMap["/r2/left_arm/joint0"]->getActualStates().controlMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::OFF,  jointControlManagerMap["/r2/neck/joint0"]->getActualStates().controlMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::PARK, jointControlManagerMap["/r2/left_leg/joint0"]->getCommandStates().controlMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::PARK, jointControlManagerMap["/r2/right_leg/gripper/joint0"]->getCommandStates().controlMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::PARK, jointControlManagerMap["/r2/left_arm/joint0"]->getCommandStates().controlMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::PARK, jointControlManagerMap["/r2/neck/joint0"]->getCommandStates().controlMode.state);
}

TEST_F(JointControlManagerTest, VerifyCommandModeState)
{
    // Fake coeffs loaded and MultiLoop
    nodeRegisterManager->nodeRegisterMap["/r2/left_leg/joint0"].status["/r2/left_leg/joint0/StatReg2"].registerValue                   = 0x0103; // 0000 0001 0000 0011
    nodeRegisterManager->nodeRegisterMap["/r2/right_leg/gripper/joint0"].status["/r2/right_leg/gripper/joint0/StatReg2"].registerValue = 0x0103; // 0000 0001 0000 0011
    nodeRegisterManager->nodeRegisterMap["/r2/left_arm/joint0"].status["/r2/left_arm/joint0/StatReg2"].registerValue                   = 0x0103; // 0000 0001 0000 0011
    nodeRegisterManager->nodeRegisterMap["/r2/neck/joint0"].status["/r2/neck/joint0/StatReg2"].registerValue                           = 0x0103; // 0000 0001 0000 0011

    usleep(timeLimit*1000000);
    EXPECT_TRUE(jointControlManagerMap["/r2/left_leg/joint0"]->verifyStates());
    EXPECT_TRUE(jointControlManagerMap["/r2/right_leg/gripper/joint0"]->verifyStates());
    EXPECT_TRUE(jointControlManagerMap["/r2/left_arm/joint0"]->verifyStates());
    EXPECT_TRUE(jointControlManagerMap["/r2/neck/joint0"]->verifyStates());

    commandMsg.commandMode.state = nasa_r2_common_msgs::JointControlCommandMode::MULTILOOPSTEP;
    jointControlManagerMap["/r2/left_leg/joint0"]->setCommandStates(commandMsg);
    jointControlManagerMap["/r2/right_leg/gripper/joint0"]->setCommandStates(commandMsg);
    jointControlManagerMap["/r2/left_arm/joint0"]->setCommandStates(commandMsg);
    jointControlManagerMap["/r2/neck/joint0"]->setCommandStates(commandMsg);

    usleep(timeLimit*1000000);
    EXPECT_FALSE(jointControlManagerMap["/r2/left_leg/joint0"]->verifyStates());
    EXPECT_FALSE(jointControlManagerMap["/r2/right_leg/gripper/joint0"]->verifyStates());
    EXPECT_FALSE(jointControlManagerMap["/r2/left_arm/joint0"]->verifyStates());
    EXPECT_FALSE(jointControlManagerMap["/r2/neck/joint0"]->verifyStates());
    EXPECT_TRUE(jointControlManagerMap["/r2/left_leg/joint0"]->verifyStates());
    EXPECT_TRUE(jointControlManagerMap["/r2/right_leg/gripper/joint0"]->verifyStates());
    EXPECT_TRUE(jointControlManagerMap["/r2/left_arm/joint0"]->verifyStates());
    EXPECT_TRUE(jointControlManagerMap["/r2/neck/joint0"]->verifyStates());

    EXPECT_EQ(nasa_r2_common_msgs::JointControlCommandMode::MOTCOM,        jointControlManagerMap["/r2/left_leg/joint0"]->getActualStates().commandMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlCommandMode::MOTCOM,        jointControlManagerMap["/r2/right_leg/gripper/joint0"]->getActualStates().commandMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlCommandMode::MOTCOM,        jointControlManagerMap["/r2/left_arm/joint0"]->getActualStates().commandMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlCommandMode::MOTCOM,        jointControlManagerMap["/r2/neck/joint0"]->getActualStates().commandMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlCommandMode::MULTILOOPSTEP, jointControlManagerMap["/r2/left_leg/joint0"]->getCommandStates().commandMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlCommandMode::MULTILOOPSTEP, jointControlManagerMap["/r2/right_leg/gripper/joint0"]->getCommandStates().commandMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlCommandMode::MULTILOOPSTEP, jointControlManagerMap["/r2/left_arm/joint0"]->getCommandStates().commandMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlCommandMode::MULTILOOPSTEP, jointControlManagerMap["/r2/neck/joint0"]->getCommandStates().commandMode.state);
}

TEST_F(JointControlManagerTest, VerifyCalibrationModeState)
{
    usleep(timeLimit*1000000 + 10000);
    EXPECT_TRUE(jointControlManagerMap["/r2/left_leg/joint0"]->verifyStates());
    EXPECT_TRUE(jointControlManagerMap["/r2/right_leg/gripper/joint0"]->verifyStates());
    EXPECT_TRUE(jointControlManagerMap["/r2/left_arm/joint0"]->verifyStates());
    EXPECT_TRUE(jointControlManagerMap["/r2/neck/joint0"]->verifyStates());

    EXPECT_EQ(nasa_r2_common_msgs::JointControlCalibrationMode::DISABLE, jointControlManagerMap["/r2/left_leg/joint0"]->getActualStates().calibrationMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlCalibrationMode::DISABLE, jointControlManagerMap["/r2/right_leg/gripper/joint0"]->getActualStates().calibrationMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlCalibrationMode::DISABLE, jointControlManagerMap["/r2/left_arm/joint0"]->getActualStates().calibrationMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlCalibrationMode::DISABLE, jointControlManagerMap["/r2/neck/joint0"]->getActualStates().calibrationMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlCalibrationMode::DISABLE, jointControlManagerMap["/r2/left_leg/joint0"]->getCommandStates().calibrationMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlCalibrationMode::DISABLE, jointControlManagerMap["/r2/right_leg/gripper/joint0"]->getCommandStates().calibrationMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlCalibrationMode::DISABLE, jointControlManagerMap["/r2/left_arm/joint0"]->getCommandStates().calibrationMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlCalibrationMode::DISABLE, jointControlManagerMap["/r2/neck/joint0"]->getCommandStates().calibrationMode.state);

    commandMsg.calibrationMode.state = nasa_r2_common_msgs::JointControlCalibrationMode::ENABLE;
    jointControlManagerMap["/r2/left_leg/joint0"]->setCommandStates(commandMsg);
    jointControlManagerMap["/r2/right_leg/gripper/joint0"]->setCommandStates(commandMsg);
    jointControlManagerMap["/r2/left_arm/joint0"]->setCommandStates(commandMsg);
    jointControlManagerMap["/r2/neck/joint0"]->setCommandStates(commandMsg);

    usleep(timeLimit*1000000);
    EXPECT_FALSE(jointControlManagerMap["/r2/left_leg/joint0"]->verifyStates());
    EXPECT_FALSE(jointControlManagerMap["/r2/right_leg/gripper/joint0"]->verifyStates());
    EXPECT_FALSE(jointControlManagerMap["/r2/left_arm/joint0"]->verifyStates());
    EXPECT_FALSE(jointControlManagerMap["/r2/neck/joint0"]->verifyStates());
    EXPECT_TRUE(jointControlManagerMap["/r2/left_leg/joint0"]->verifyStates());
    EXPECT_TRUE(jointControlManagerMap["/r2/right_leg/gripper/joint0"]->verifyStates());
    EXPECT_TRUE(jointControlManagerMap["/r2/left_arm/joint0"]->verifyStates());
    EXPECT_TRUE(jointControlManagerMap["/r2/neck/joint0"]->verifyStates());

    EXPECT_EQ(nasa_r2_common_msgs::JointControlCalibrationMode::DISABLE, jointControlManagerMap["/r2/left_leg/joint0"]->getActualStates().calibrationMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlCalibrationMode::DISABLE, jointControlManagerMap["/r2/right_leg/gripper/joint0"]->getActualStates().calibrationMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlCalibrationMode::DISABLE, jointControlManagerMap["/r2/left_arm/joint0"]->getActualStates().calibrationMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlCalibrationMode::DISABLE, jointControlManagerMap["/r2/neck/joint0"]->getActualStates().calibrationMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlCalibrationMode::ENABLE,  jointControlManagerMap["/r2/left_leg/joint0"]->getCommandStates().calibrationMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlCalibrationMode::ENABLE,  jointControlManagerMap["/r2/right_leg/gripper/joint0"]->getCommandStates().calibrationMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlCalibrationMode::ENABLE,  jointControlManagerMap["/r2/left_arm/joint0"]->getCommandStates().calibrationMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlCalibrationMode::ENABLE,  jointControlManagerMap["/r2/neck/joint0"]->getCommandStates().calibrationMode.state);
}

int main(int argc, char **argv){
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
