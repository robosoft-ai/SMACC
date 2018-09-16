#include <iostream>
#include <gtest/gtest.h>
#include <boost/shared_ptr.hpp>
#include "nasa_r2_config_core/RobotInstanceFactory.h"
#include "nasa_r2_config_core/NodeRegisterManager.h"
#include "nasa_robodyn_mechanisms_core/JointControlActualFsmGripper.h"
#include "nasa_robodyn_mechanisms_core/JointControlActualFsmSeriesElastic.h"
#include "nasa_robodyn_mechanisms_core/JointControlActualFsmWrist.h"
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

class JointControlActualFsmTest : public ::testing::Test {
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
                    jointControlFsmMap[mechanism] = boost::make_shared<JointControlActualFsmSeriesElastic>(mechanism, io, nodeRegisterManager);
                    nodeRegisterManager->nodeRegisterMap[mechanism].status[mechanism+"/StatReg1"].registerValue = 0x9000; // 1001 0000 0000 0000
                }
                else if (jointType == "Rigid")
                {
                    nodeRegisterManager->addNode(mechanism, fileName);
                    jointControlFsmMap[mechanism] = boost::make_shared<JointControlActualFsmSeriesElastic>(mechanism, io, nodeRegisterManager);
                    nodeRegisterManager->nodeRegisterMap[mechanism].status[mechanism+"/StatReg1"].registerValue = 0x9000; // 1001 0000 0000 0000
                }
                else if (jointType == "Gripper")
                {
                    nodeRegisterManager->addNode(mechanism, fileName);
                    jointControlFsmMap[mechanism] = boost::make_shared<JointControlActualFsmGripper>(mechanism, io, nodeRegisterManager);
                    nodeRegisterManager->nodeRegisterMap[mechanism].status[mechanism+"/StatReg1"].registerValue = 0x9000; // 1001 0000 0000 0000
                }

                // Complex types
                else if (jointType == "Wrist")
                {
                    jointControlFsmMap[mechanism] = boost::make_shared<JointControlActualFsmWrist>(mechanism, io);
                    io.setLiveCoeff(StringUtilities::makeFullyQualifiedRoboDynElement(mechanism, "CalibrationState"),      nasa_r2_common_msgs::JointControlCalibrationMode::IGNORE);
                    io.setLiveCoeff(StringUtilities::makeFullyQualifiedRoboDynElement(mechanism, "PitchLimit"),            0);
                    io.setLiveCoeff(StringUtilities::makeFullyQualifiedRoboDynElement(mechanism, "YawLimit"),              0);
                    io.setLiveCoeff(StringUtilities::makeFullyQualifiedRoboDynElement(mechanism, "LittlesideSliderLimit"), 0);
                    io.setLiveCoeff(StringUtilities::makeFullyQualifiedRoboDynElement(mechanism, "ThumbsideSliderLimit"),  0);
                    io.setLiveCoeff(StringUtilities::makeFullyQualifiedRoboDynElement(mechanism, "SensorError"),           0);
                    io.setLiveCoeff(StringUtilities::makeFullyQualifiedRoboDynElement(mechanism, "SliderDiffError"),       0);
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
                    RCS::Logger::log("gov.nasa.robonet.JointControlActualFsmTest", log4cpp::Priority::ERROR, err.str());
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
        std::map< std::string, boost::shared_ptr<JointControlActualFsmInterface> > jointControlFsmMap;
        JointControlCommonInterface::IoFunctions io;
};

TEST_F(JointControlActualFsmTest, InitialBootLoader)
{
    jointControlFsmMap["/r2/left_leg/joint0"]->updateControlModeState();
    jointControlFsmMap["/r2/right_leg/gripper/joint0"]->updateControlModeState();
    jointControlFsmMap["/r2/left_arm/joint0"]->updateControlModeState();
    jointControlFsmMap["/r2/neck/joint0"]->updateControlModeState();
    jointControlFsmMap["/r2/right_arm/wrist"]->updateControlModeState();

    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::BOOTLOADER, jointControlFsmMap["/r2/left_leg/joint0"]->getStates().controlMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::BOOTLOADER, jointControlFsmMap["/r2/right_leg/gripper/joint0"]->getStates().controlMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::BOOTLOADER, jointControlFsmMap["/r2/left_arm/joint0"]->getStates().controlMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::BOOTLOADER, jointControlFsmMap["/r2/neck/joint0"]->getStates().controlMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::BOOTLOADER, jointControlFsmMap["/r2/right_arm/wrist"]->getStates().controlMode.state);
}

TEST_F(JointControlActualFsmTest, FpgaFaults)
{
    // Fake CommAlive is dead
    nodeRegisterManager->nodeRegisterMap["/r2/left_leg/joint0"].status["/r2/left_leg/joint0/StatReg1"].registerValue                   = 0x8000; // 1000 0000 0000 0000
    nodeRegisterManager->nodeRegisterMap["/r2/right_leg/gripper/joint0"].status["/r2/right_leg/gripper/joint0/StatReg1"].registerValue = 0x8000; // 1000 0000 0000 0000
    nodeRegisterManager->nodeRegisterMap["/r2/left_arm/joint0"].status["/r2/left_arm/joint0/StatReg1"].registerValue                   = 0x8000; // 1000 0000 0000 0000
    nodeRegisterManager->nodeRegisterMap["/r2/neck/joint0"].status["/r2/neck/joint0/StatReg1"].registerValue                           = 0x8000; // 1000 0000 0000 0000

    jointControlFsmMap["/r2/left_leg/joint0"]->updateControlModeState();
    jointControlFsmMap["/r2/right_leg/gripper/joint0"]->updateControlModeState();
    jointControlFsmMap["/r2/left_arm/joint0"]->updateControlModeState();
    jointControlFsmMap["/r2/neck/joint0"]->updateControlModeState();

    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::FAULTED, jointControlFsmMap["/r2/left_leg/joint0"]->getStates().controlMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::FAULTED, jointControlFsmMap["/r2/right_leg/gripper/joint0"]->getStates().controlMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::FAULTED, jointControlFsmMap["/r2/left_arm/joint0"]->getStates().controlMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::FAULTED, jointControlFsmMap["/r2/neck/joint0"]->getStates().controlMode.state);

    // UnFake CommAlive is dead
    nodeRegisterManager->nodeRegisterMap["/r2/left_leg/joint0"].status["/r2/left_leg/joint0/StatReg1"].registerValue                   = 0x9000; // 1001 0000 0000 0000
    nodeRegisterManager->nodeRegisterMap["/r2/right_leg/gripper/joint0"].status["/r2/right_leg/gripper/joint0/StatReg1"].registerValue = 0x9000; // 1001 0000 0000 0000
    nodeRegisterManager->nodeRegisterMap["/r2/left_arm/joint0"].status["/r2/left_arm/joint0/StatReg1"].registerValue                   = 0x9000; // 1001 0000 0000 0000
    nodeRegisterManager->nodeRegisterMap["/r2/neck/joint0"].status["/r2/neck/joint0/StatReg1"].registerValue                           = 0x9000; // 1001 0000 0000 0000

    jointControlFsmMap["/r2/left_leg/joint0"]->updateControlModeState();
    jointControlFsmMap["/r2/right_leg/gripper/joint0"]->updateControlModeState();
    jointControlFsmMap["/r2/left_arm/joint0"]->updateControlModeState();
    jointControlFsmMap["/r2/neck/joint0"]->updateControlModeState();

    // Fake ProcAlive is dead
    nodeRegisterManager->nodeRegisterMap["/r2/left_leg/joint0"].status["/r2/left_leg/joint0/StatReg1"].registerValue                   = 0x1000; // 0001 0000 0000 0000
    nodeRegisterManager->nodeRegisterMap["/r2/right_leg/gripper/joint0"].status["/r2/right_leg/gripper/joint0/StatReg1"].registerValue = 0x1000; // 0001 0000 0000 0000
    nodeRegisterManager->nodeRegisterMap["/r2/left_arm/joint0"].status["/r2/left_arm/joint0/StatReg1"].registerValue                   = 0x1000; // 0001 0000 0000 0000
    nodeRegisterManager->nodeRegisterMap["/r2/neck/joint0"].status["/r2/neck/joint0/StatReg1"].registerValue                           = 0x1000; // 0001 0000 0000 0000

    jointControlFsmMap["/r2/left_leg/joint0"]->updateControlModeState();
    jointControlFsmMap["/r2/right_leg/gripper/joint0"]->updateControlModeState();
    jointControlFsmMap["/r2/left_arm/joint0"]->updateControlModeState();
    jointControlFsmMap["/r2/neck/joint0"]->updateControlModeState();

    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::FAULTED, jointControlFsmMap["/r2/left_leg/joint0"]->getStates().controlMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::FAULTED, jointControlFsmMap["/r2/right_leg/gripper/joint0"]->getStates().controlMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::FAULTED, jointControlFsmMap["/r2/left_arm/joint0"]->getStates().controlMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::FAULTED, jointControlFsmMap["/r2/neck/joint0"]->getStates().controlMode.state);

    // UnFake ProcAlive is dead
    nodeRegisterManager->nodeRegisterMap["/r2/left_leg/joint0"].status["/r2/left_leg/joint0/StatReg1"].registerValue                   = 0x9000; // 1001 0000 0000 0000
    nodeRegisterManager->nodeRegisterMap["/r2/right_leg/gripper/joint0"].status["/r2/right_leg/gripper/joint0/StatReg1"].registerValue = 0x9000; // 1001 0000 0000 0000
    nodeRegisterManager->nodeRegisterMap["/r2/left_arm/joint0"].status["/r2/left_arm/joint0/StatReg1"].registerValue                   = 0x9000; // 1001 0000 0000 0000
    nodeRegisterManager->nodeRegisterMap["/r2/neck/joint0"].status["/r2/neck/joint0/StatReg1"].registerValue                           = 0x9000; // 1001 0000 0000 0000

    jointControlFsmMap["/r2/left_leg/joint0"]->updateControlModeState();
    jointControlFsmMap["/r2/right_leg/gripper/joint0"]->updateControlModeState();
    jointControlFsmMap["/r2/left_arm/joint0"]->updateControlModeState();
    jointControlFsmMap["/r2/neck/joint0"]->updateControlModeState();
}

TEST_F(JointControlActualFsmTest, Off)
{
    nodeRegisterManager->setControlValue("/r2/left_leg/joint0",          "BootEnable",1);
    nodeRegisterManager->setControlValue("/r2/right_leg/gripper/joint0", "BootEnable",1);
    nodeRegisterManager->setControlValue("/r2/left_arm/joint0",          "BootEnable",1);
    nodeRegisterManager->setControlValue("/r2/neck/joint0",              "BootEnable",1);

    jointControlFsmMap["/r2/left_leg/joint0"]->updateControlModeState();
    jointControlFsmMap["/r2/right_leg/gripper/joint0"]->updateControlModeState();
    jointControlFsmMap["/r2/left_arm/joint0"]->updateControlModeState();
    jointControlFsmMap["/r2/neck/joint0"]->updateControlModeState();

    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::OFF,  jointControlFsmMap["/r2/left_leg/joint0"]->getStates().controlMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::OFF,  jointControlFsmMap["/r2/right_leg/gripper/joint0"]->getStates().controlMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::OFF,  jointControlFsmMap["/r2/left_arm/joint0"]->getStates().controlMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::OFF,  jointControlFsmMap["/r2/neck/joint0"]->getStates().controlMode.state);
}

TEST_F(JointControlActualFsmTest, FaultIfInMotCom)
{
    // Fake BootEnable
    nodeRegisterManager->setControlValue("/r2/left_leg/joint0",          "BootEnable",1);
    nodeRegisterManager->setControlValue("/r2/right_leg/gripper/joint0", "BootEnable",1);
    nodeRegisterManager->setControlValue("/r2/left_arm/joint0",          "BootEnable",1);
    nodeRegisterManager->setControlValue("/r2/neck/joint0",              "BootEnable",1);

    // Fake Embedded MotCom and BridgeEnable
    nodeRegisterManager->nodeRegisterMap["/r2/left_leg/joint0"].status["/r2/left_leg/joint0/StatReg1"].registerValue                   = 0x9010; // 1001 0000 0001 0000
    nodeRegisterManager->nodeRegisterMap["/r2/right_leg/gripper/joint0"].status["/r2/right_leg/gripper/joint0/StatReg1"].registerValue = 0x9010; // 1001 0000 0001 0000
    nodeRegisterManager->nodeRegisterMap["/r2/left_arm/joint0"].status["/r2/left_arm/joint0/StatReg1"].registerValue                   = 0x9010; // 1001 0000 0001 0000
    nodeRegisterManager->nodeRegisterMap["/r2/neck/joint0"].status["/r2/neck/joint0/StatReg1"].registerValue                           = 0x9010; // 1001 0000 0001 0000

    // Fake coeffs loaded and joint faulted
    nodeRegisterManager->nodeRegisterMap["/r2/left_leg/joint0"].status["/r2/left_leg/joint0/StatReg2"].registerValue                   = 0x0180; // 0000 0001 1000 0000
    nodeRegisterManager->nodeRegisterMap["/r2/right_leg/gripper/joint0"].status["/r2/right_leg/gripper/joint0/StatReg2"].registerValue = 0x0180; // 0000 0001 1000 0000
    nodeRegisterManager->nodeRegisterMap["/r2/left_arm/joint0"].status["/r2/left_arm/joint0/StatReg2"].registerValue                   = 0x0180; // 0000 0001 1000 0000
    nodeRegisterManager->nodeRegisterMap["/r2/neck/joint0"].status["/r2/neck/joint0/StatReg2"].registerValue                           = 0x0180; // 0000 0001 1000 0000

    jointControlFsmMap["/r2/left_leg/joint0"]->updateControlModeState();
    jointControlFsmMap["/r2/right_leg/gripper/joint0"]->updateControlModeState();
    jointControlFsmMap["/r2/left_arm/joint0"]->updateControlModeState();
    jointControlFsmMap["/r2/neck/joint0"]->updateControlModeState();

    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::FAULTED, jointControlFsmMap["/r2/left_leg/joint0"]->getStates().controlMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::FAULTED, jointControlFsmMap["/r2/right_leg/gripper/joint0"]->getStates().controlMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::FAULTED, jointControlFsmMap["/r2/left_arm/joint0"]->getStates().controlMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::FAULTED, jointControlFsmMap["/r2/neck/joint0"]->getStates().controlMode.state);
}

TEST_F(JointControlActualFsmTest, Faults)
{
    // Fake getting into MultiLoopStep
    nodeRegisterManager->nodeRegisterMap["/r2/left_leg/joint0"].status["/r2/left_leg/joint0/StatReg2"].registerValue                   = 0x0103; // 0000 0001 0000 0011
    nodeRegisterManager->nodeRegisterMap["/r2/right_leg/gripper/joint0"].status["/r2/right_leg/gripper/joint0/StatReg2"].registerValue = 0x0103; // 0000 0001 0000 0011
    nodeRegisterManager->nodeRegisterMap["/r2/left_arm/joint0"].status["/r2/left_arm/joint0/StatReg2"].registerValue                   = 0x0103; // 0000 0001 0000 0011
    nodeRegisterManager->nodeRegisterMap["/r2/neck/joint0"].status["/r2/neck/joint0/StatReg2"].registerValue                           = 0x0103; // 0000 0001 0000 0011
    nodeRegisterManager->nodeRegisterMap["/r2/left_leg/joint0"].status["/r2/left_leg/joint0/StatReg1"].registerValue                   = 0x9040; // 1001 0000 0100 0000
    nodeRegisterManager->nodeRegisterMap["/r2/right_leg/gripper/joint0"].status["/r2/right_leg/gripper/joint0/StatReg1"].registerValue = 0x9040; // 1001 0000 0100 0000
    nodeRegisterManager->nodeRegisterMap["/r2/left_arm/joint0"].status["/r2/left_arm/joint0/StatReg1"].registerValue                   = 0x9040; // 1001 0000 0100 0000
    nodeRegisterManager->nodeRegisterMap["/r2/neck/joint0"].status["/r2/neck/joint0/StatReg1"].registerValue                           = 0x9040; // 1001 0000 0100 0000
    jointControlFsmMap["/r2/left_leg/joint0"]->updateCommandModeState();
    jointControlFsmMap["/r2/right_leg/gripper/joint0"]->updateCommandModeState();
    jointControlFsmMap["/r2/left_arm/joint0"]->updateCommandModeState();
    jointControlFsmMap["/r2/neck/joint0"]->updateCommandModeState();
    EXPECT_EQ(nasa_r2_common_msgs::JointControlCommandMode::MULTILOOPSTEP, jointControlFsmMap["/r2/left_leg/joint0"]->getStates().commandMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlCommandMode::MULTILOOPSTEP, jointControlFsmMap["/r2/right_leg/gripper/joint0"]->getStates().commandMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlCommandMode::MULTILOOPSTEP, jointControlFsmMap["/r2/left_arm/joint0"]->getStates().commandMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlCommandMode::MULTILOOPSTEP, jointControlFsmMap["/r2/neck/joint0"]->getStates().commandMode.state);

    // Fake BootEnable
    nodeRegisterManager->setControlValue("/r2/left_leg/joint0",          "BootEnable",1);
    nodeRegisterManager->setControlValue("/r2/right_leg/gripper/joint0", "BootEnable",1);
    nodeRegisterManager->setControlValue("/r2/left_arm/joint0",          "BootEnable",1);
    nodeRegisterManager->setControlValue("/r2/neck/joint0",              "BootEnable",1);

    // Fake Embedded MotCom and BridgeEnable
    nodeRegisterManager->nodeRegisterMap["/r2/left_leg/joint0"].status["/r2/left_leg/joint0/StatReg1"].registerValue                   = 0x9050; // 1001 0000 0101 0000
    nodeRegisterManager->nodeRegisterMap["/r2/right_leg/gripper/joint0"].status["/r2/right_leg/gripper/joint0/StatReg1"].registerValue = 0x9050; // 1001 0000 0101 0000
    nodeRegisterManager->nodeRegisterMap["/r2/left_arm/joint0"].status["/r2/left_arm/joint0/StatReg1"].registerValue                   = 0x9050; // 1001 0000 0101 0000
    nodeRegisterManager->nodeRegisterMap["/r2/neck/joint0"].status["/r2/neck/joint0/StatReg1"].registerValue                           = 0x9050; // 1001 0000 0101 0000

    // Fake coeffs loaded and joint faulted
    nodeRegisterManager->nodeRegisterMap["/r2/left_leg/joint0"].status["/r2/left_leg/joint0/StatReg2"].registerValue                   = 0x0180; // 0000 0001 1000 0000
    nodeRegisterManager->nodeRegisterMap["/r2/right_leg/gripper/joint0"].status["/r2/right_leg/gripper/joint0/StatReg2"].registerValue = 0x0180; // 0000 0001 1000 0000
    nodeRegisterManager->nodeRegisterMap["/r2/left_arm/joint0"].status["/r2/left_arm/joint0/StatReg2"].registerValue                   = 0x0180; // 0000 0001 1000 0000
    nodeRegisterManager->nodeRegisterMap["/r2/neck/joint0"].status["/r2/neck/joint0/StatReg2"].registerValue                           = 0x0180; // 0000 0001 1000 0000

    // Fake complex calibrated and faulted
    io.setLiveCoeff(StringUtilities::makeFullyQualifiedRoboDynElement("/r2/right_arm/wrist", "IsCalibrated"), 1);
    io.setLiveCoeff(StringUtilities::makeFullyQualifiedRoboDynElement("/r2/right_arm/wrist", "PitchLimit"),   1);

    jointControlFsmMap["/r2/left_leg/joint0"]->updateControlModeState();
    jointControlFsmMap["/r2/right_leg/gripper/joint0"]->updateControlModeState();
    jointControlFsmMap["/r2/left_arm/joint0"]->updateControlModeState();
    jointControlFsmMap["/r2/neck/joint0"]->updateControlModeState();
    jointControlFsmMap["/r2/right_arm/wrist"]->updateControlModeState();

    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::FAULTED, jointControlFsmMap["/r2/left_leg/joint0"]->getStates().controlMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::FAULTED, jointControlFsmMap["/r2/right_leg/gripper/joint0"]->getStates().controlMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::FAULTED, jointControlFsmMap["/r2/left_arm/joint0"]->getStates().controlMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::FAULTED, jointControlFsmMap["/r2/neck/joint0"]->getStates().controlMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::FAULTED, jointControlFsmMap["/r2/right_arm/wrist"]->getStates().controlMode.state);
}

TEST_F(JointControlActualFsmTest, Park)
{
    nodeRegisterManager->setControlValue("/r2/left_leg/joint0",          "BootEnable",1);
    nodeRegisterManager->setControlValue("/r2/right_leg/gripper/joint0", "BootEnable",1);
    nodeRegisterManager->setControlValue("/r2/left_arm/joint0",          "BootEnable",1);
    nodeRegisterManager->setControlValue("/r2/neck/joint0",              "BootEnable",1);

    nodeRegisterManager->nodeRegisterMap["/r2/left_leg/joint0"].status["/r2/left_leg/joint0/StatReg1"].registerValue                   = 0x9011; // 1001 0000 0001 0001
    nodeRegisterManager->nodeRegisterMap["/r2/right_leg/gripper/joint0"].status["/r2/right_leg/gripper/joint0/StatReg1"].registerValue = 0x9011; // 1001 0000 0001 0001
    nodeRegisterManager->nodeRegisterMap["/r2/left_arm/joint0"].status["/r2/left_arm/joint0/StatReg1"].registerValue                   = 0x9011; // 1001 0000 0001 0001
    nodeRegisterManager->nodeRegisterMap["/r2/neck/joint0"].status["/r2/neck/joint0/StatReg1"].registerValue                           = 0x9011; // 1001 0000 0001 0001

    jointControlFsmMap["/r2/left_leg/joint0"]->updateControlModeState();
    jointControlFsmMap["/r2/right_leg/gripper/joint0"]->updateControlModeState();
    jointControlFsmMap["/r2/left_arm/joint0"]->updateControlModeState();
    jointControlFsmMap["/r2/neck/joint0"]->updateControlModeState();

    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::PARK, jointControlFsmMap["/r2/left_leg/joint0"]->getStates().controlMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::PARK, jointControlFsmMap["/r2/right_leg/gripper/joint0"]->getStates().controlMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::PARK, jointControlFsmMap["/r2/left_arm/joint0"]->getStates().controlMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::PARK, jointControlFsmMap["/r2/neck/joint0"]->getStates().controlMode.state);
}

TEST_F(JointControlActualFsmTest, Neutral)
{
    nodeRegisterManager->setControlValue("/r2/left_leg/joint0",          "BootEnable",1);
    nodeRegisterManager->setControlValue("/r2/right_leg/gripper/joint0", "BootEnable",1);
    nodeRegisterManager->setControlValue("/r2/left_arm/joint0",          "BootEnable",1);
    nodeRegisterManager->setControlValue("/r2/neck/joint0",              "BootEnable",1);

    nodeRegisterManager->nodeRegisterMap["/r2/left_leg/joint0"].status["/r2/left_leg/joint0/StatReg1"].registerValue                   = 0x9012; // 1001 0000 0001 0010
    nodeRegisterManager->nodeRegisterMap["/r2/right_leg/gripper/joint0"].status["/r2/right_leg/gripper/joint0/StatReg1"].registerValue = 0x9012; // 1001 0000 0001 0010
    nodeRegisterManager->nodeRegisterMap["/r2/left_arm/joint0"].status["/r2/left_arm/joint0/StatReg1"].registerValue                   = 0x9012; // 1001 0000 0001 0010
    nodeRegisterManager->nodeRegisterMap["/r2/neck/joint0"].status["/r2/neck/joint0/StatReg1"].registerValue                           = 0x9012; // 1001 0000 0001 0010

    jointControlFsmMap["/r2/left_leg/joint0"]->updateControlModeState();
    jointControlFsmMap["/r2/right_leg/gripper/joint0"]->updateControlModeState();
    jointControlFsmMap["/r2/left_arm/joint0"]->updateControlModeState();
    jointControlFsmMap["/r2/neck/joint0"]->updateControlModeState();

    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::NEUTRAL, jointControlFsmMap["/r2/left_leg/joint0"]->getStates().controlMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::NEUTRAL, jointControlFsmMap["/r2/right_leg/gripper/joint0"]->getStates().controlMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::NEUTRAL, jointControlFsmMap["/r2/left_arm/joint0"]->getStates().controlMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::NEUTRAL, jointControlFsmMap["/r2/neck/joint0"]->getStates().controlMode.state);
}

TEST_F(JointControlActualFsmTest, Drive)
{
    nodeRegisterManager->setControlValue("/r2/left_leg/joint0",          "BootEnable",1);
    nodeRegisterManager->setControlValue("/r2/right_leg/gripper/joint0", "BootEnable",1);
    nodeRegisterManager->setControlValue("/r2/left_arm/joint0",          "BootEnable",1);
    nodeRegisterManager->setControlValue("/r2/neck/joint0",              "BootEnable",1);

    io.setLiveCoeff("/r2/right_arm/wrist/CalibrationState", nasa_r2_common_msgs::JointControlCalibrationMode::ENABLE);

    nodeRegisterManager->nodeRegisterMap["/r2/left_leg/joint0"].status["/r2/left_leg/joint0/StatReg1"].registerValue                   = 0x9013; // 1001 0000 0001 0011
    nodeRegisterManager->nodeRegisterMap["/r2/right_leg/gripper/joint0"].status["/r2/right_leg/gripper/joint0/StatReg1"].registerValue = 0x9013; // 1001 0000 0001 0011
    nodeRegisterManager->nodeRegisterMap["/r2/left_arm/joint0"].status["/r2/left_arm/joint0/StatReg1"].registerValue                   = 0x9013; // 1001 0000 0001 0011
    nodeRegisterManager->nodeRegisterMap["/r2/neck/joint0"].status["/r2/neck/joint0/StatReg1"].registerValue                           = 0x9013; // 1001 0000 0001 0011

    jointControlFsmMap["/r2/left_leg/joint0"]->updateControlModeState();
    jointControlFsmMap["/r2/right_leg/gripper/joint0"]->updateControlModeState();
    jointControlFsmMap["/r2/left_arm/joint0"]->updateControlModeState();
    jointControlFsmMap["/r2/neck/joint0"]->updateControlModeState();
    jointControlFsmMap["/r2/right_arm/wrist"]->updateControlModeState();

    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::DRIVE, jointControlFsmMap["/r2/left_leg/joint0"]->getStates().controlMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::DRIVE, jointControlFsmMap["/r2/right_leg/gripper/joint0"]->getStates().controlMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::DRIVE, jointControlFsmMap["/r2/left_arm/joint0"]->getStates().controlMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::DRIVE, jointControlFsmMap["/r2/neck/joint0"]->getStates().controlMode.state);
    EXPECT_NE(nasa_r2_common_msgs::JointControlMode::DRIVE, jointControlFsmMap["/r2/right_arm/wrist"]->getStates().controlMode.state);

}

TEST_F(JointControlActualFsmTest, AFewControlModeInvalid)
{
    nodeRegisterManager->setControlValue("/r2/left_leg/joint0",          "BootEnable", 1);
    nodeRegisterManager->setControlValue("/r2/right_leg/gripper/joint0", "BootEnable", 1);
    nodeRegisterManager->setControlValue("/r2/left_arm/joint0",          "BootEnable", 1);
    nodeRegisterManager->setControlValue("/r2/neck/joint0",              "BootEnable", 1);

    // MotorEnable while BridgeDisable
    nodeRegisterManager->nodeRegisterMap["/r2/left_leg/joint0"].status["/r2/left_leg/joint0/StatReg1"].registerValue                   = 0x9001; // 1001 0000 0000 0001
    nodeRegisterManager->nodeRegisterMap["/r2/right_leg/gripper/joint0"].status["/r2/right_leg/gripper/joint0/StatReg1"].registerValue = 0x9001; // 1001 0000 0000 0001
    nodeRegisterManager->nodeRegisterMap["/r2/left_arm/joint0"].status["/r2/left_arm/joint0/StatReg1"].registerValue                   = 0x9001; // 1001 0000 0000 0001
    nodeRegisterManager->nodeRegisterMap["/r2/neck/joint0"].status["/r2/neck/joint0/StatReg1"].registerValue                           = 0x9001; // 1001 0000 0000 0001

    jointControlFsmMap["/r2/left_leg/joint0"]->updateControlModeState();
    jointControlFsmMap["/r2/right_leg/gripper/joint0"]->updateControlModeState();
    jointControlFsmMap["/r2/left_arm/joint0"]->updateControlModeState();
    jointControlFsmMap["/r2/neck/joint0"]->updateControlModeState();

    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::INVALID, jointControlFsmMap["/r2/left_leg/joint0"]->getStates().controlMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::INVALID, jointControlFsmMap["/r2/right_leg/gripper/joint0"]->getStates().controlMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::INVALID, jointControlFsmMap["/r2/left_arm/joint0"]->getStates().controlMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::INVALID, jointControlFsmMap["/r2/neck/joint0"]->getStates().controlMode.state);

    // Reset
    nodeRegisterManager->nodeRegisterMap["/r2/left_leg/joint0"].status["/r2/left_leg/joint0/StatReg1"].registerValue                   = 0x9000; // 1001 0000 0000 0000
    nodeRegisterManager->nodeRegisterMap["/r2/right_leg/gripper/joint0"].status["/r2/right_leg/gripper/joint0/StatReg1"].registerValue = 0x9000; // 1001 0000 0000 0000
    nodeRegisterManager->nodeRegisterMap["/r2/left_arm/joint0"].status["/r2/left_arm/joint0/StatReg1"].registerValue                   = 0x9000; // 1001 0000 0000 0000
    nodeRegisterManager->nodeRegisterMap["/r2/neck/joint0"].status["/r2/neck/joint0/StatReg1"].registerValue                           = 0x9000; // 1001 0000 0000 0000

    jointControlFsmMap["/r2/left_leg/joint0"]->updateControlModeState();
    jointControlFsmMap["/r2/right_leg/gripper/joint0"]->updateControlModeState();
    jointControlFsmMap["/r2/left_arm/joint0"]->updateControlModeState();
    jointControlFsmMap["/r2/neck/joint0"]->updateControlModeState();
}

TEST_F(JointControlActualFsmTest, InitMotCom)
{
    EXPECT_EQ(nasa_r2_common_msgs::JointControlCommandMode::MOTCOM, jointControlFsmMap["/r2/left_leg/joint0"]->getStates().commandMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlCommandMode::MOTCOM, jointControlFsmMap["/r2/right_leg/gripper/joint0"]->getStates().commandMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlCommandMode::MOTCOM, jointControlFsmMap["/r2/left_arm/joint0"]->getStates().commandMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlCommandMode::MOTCOM, jointControlFsmMap["/r2/neck/joint0"]->getStates().commandMode.state);
}

TEST_F(JointControlActualFsmTest, CantMultiLoopWithNoCoeffs)
{
    // Fake coeffs not loaded and MultiLoop
    nodeRegisterManager->nodeRegisterMap["/r2/left_leg/joint0"].status["/r2/left_leg/joint0/StatReg2"].registerValue                   = 0x0003; // 0000 0000 0000 0011
    nodeRegisterManager->nodeRegisterMap["/r2/right_leg/gripper/joint0"].status["/r2/right_leg/gripper/joint0/StatReg2"].registerValue = 0x0003; // 0000 0000 0000 0011
    nodeRegisterManager->nodeRegisterMap["/r2/left_arm/joint0"].status["/r2/left_arm/joint0/StatReg2"].registerValue                   = 0x0003; // 0000 0000 0000 0011
    nodeRegisterManager->nodeRegisterMap["/r2/neck/joint0"].status["/r2/neck/joint0/StatReg2"].registerValue                           = 0x0003; // 0000 0000 0000 0011

    // Fake MotComSource == 1
    nodeRegisterManager->nodeRegisterMap["/r2/left_leg/joint0"].status["/r2/left_leg/joint0/StatReg1"].registerValue                   = 0x9040; // 1001 0000 0100 0000
    nodeRegisterManager->nodeRegisterMap["/r2/right_leg/gripper/joint0"].status["/r2/right_leg/gripper/joint0/StatReg1"].registerValue = 0x9040; // 1001 0000 0100 0000
    nodeRegisterManager->nodeRegisterMap["/r2/left_arm/joint0"].status["/r2/left_arm/joint0/StatReg1"].registerValue                   = 0x9040; // 1001 0000 0100 0000
    nodeRegisterManager->nodeRegisterMap["/r2/neck/joint0"].status["/r2/neck/joint0/StatReg1"].registerValue                           = 0x9040; // 1001 0000 0100 0000

    jointControlFsmMap["/r2/left_leg/joint0"]->updateCommandModeState();
    jointControlFsmMap["/r2/right_leg/gripper/joint0"]->updateCommandModeState();
    jointControlFsmMap["/r2/left_arm/joint0"]->updateCommandModeState();
    jointControlFsmMap["/r2/neck/joint0"]->updateCommandModeState();

    EXPECT_EQ(nasa_r2_common_msgs::JointControlCommandMode::INVALID, jointControlFsmMap["/r2/left_leg/joint0"]->getStates().commandMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlCommandMode::INVALID, jointControlFsmMap["/r2/right_leg/gripper/joint0"]->getStates().commandMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlCommandMode::INVALID, jointControlFsmMap["/r2/left_arm/joint0"]->getStates().commandMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlCommandMode::INVALID, jointControlFsmMap["/r2/neck/joint0"]->getStates().commandMode.state);
}

TEST_F(JointControlActualFsmTest, StallMode)
{
    // Fake coeffs loaded and StallMode
    nodeRegisterManager->nodeRegisterMap["/r2/left_leg/joint0"].status["/r2/left_leg/joint0/StatReg2"].registerValue                   = 0x0101; // 0000 0001 0000 0001
    nodeRegisterManager->nodeRegisterMap["/r2/right_leg/gripper/joint0"].status["/r2/right_leg/gripper/joint0/StatReg2"].registerValue = 0x0101; // 0000 0001 0000 0001
    nodeRegisterManager->nodeRegisterMap["/r2/left_arm/joint0"].status["/r2/left_arm/joint0/StatReg2"].registerValue                   = 0x0101; // 0000 0001 0000 0001
    nodeRegisterManager->nodeRegisterMap["/r2/neck/joint0"].status["/r2/neck/joint0/StatReg2"].registerValue                           = 0x0101; // 0000 0001 0000 0001

    // Fake MotComSource == 1
    nodeRegisterManager->nodeRegisterMap["/r2/left_leg/joint0"].status["/r2/left_leg/joint0/StatReg1"].registerValue                   = 0x9040; // 1001 0000 0100 0000
    nodeRegisterManager->nodeRegisterMap["/r2/right_leg/gripper/joint0"].status["/r2/right_leg/gripper/joint0/StatReg1"].registerValue = 0x9040; // 1001 0000 0100 0000
    nodeRegisterManager->nodeRegisterMap["/r2/left_arm/joint0"].status["/r2/left_arm/joint0/StatReg1"].registerValue                   = 0x9040; // 1001 0000 0100 0000
    nodeRegisterManager->nodeRegisterMap["/r2/neck/joint0"].status["/r2/neck/joint0/StatReg1"].registerValue                           = 0x9040; // 1001 0000 0100 0000

    jointControlFsmMap["/r2/left_leg/joint0"]->updateCommandModeState();
    jointControlFsmMap["/r2/right_leg/gripper/joint0"]->updateCommandModeState();
    jointControlFsmMap["/r2/left_arm/joint0"]->updateCommandModeState();
    jointControlFsmMap["/r2/neck/joint0"]->updateCommandModeState();

    EXPECT_EQ(nasa_r2_common_msgs::JointControlCommandMode::STALLMODE, jointControlFsmMap["/r2/left_leg/joint0"]->getStates().commandMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlCommandMode::STALLMODE, jointControlFsmMap["/r2/right_leg/gripper/joint0"]->getStates().commandMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlCommandMode::STALLMODE, jointControlFsmMap["/r2/left_arm/joint0"]->getStates().commandMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlCommandMode::STALLMODE, jointControlFsmMap["/r2/neck/joint0"]->getStates().commandMode.state);
}

TEST_F(JointControlActualFsmTest, MultiLoopStep)
{
    // Fake coeffs loaded and MultiLoop
    nodeRegisterManager->nodeRegisterMap["/r2/left_leg/joint0"].status["/r2/left_leg/joint0/StatReg2"].registerValue                   = 0x0103; // 0000 0001 0000 0011
    nodeRegisterManager->nodeRegisterMap["/r2/right_leg/gripper/joint0"].status["/r2/right_leg/gripper/joint0/StatReg2"].registerValue = 0x0103; // 0000 0001 0000 0011
    nodeRegisterManager->nodeRegisterMap["/r2/left_arm/joint0"].status["/r2/left_arm/joint0/StatReg2"].registerValue                   = 0x0103; // 0000 0001 0000 0011
    nodeRegisterManager->nodeRegisterMap["/r2/neck/joint0"].status["/r2/neck/joint0/StatReg2"].registerValue                           = 0x0103; // 0000 0001 0000 0011

    // Fake MotComSource == 1
    nodeRegisterManager->nodeRegisterMap["/r2/left_leg/joint0"].status["/r2/left_leg/joint0/StatReg1"].registerValue                   = 0x9040; // 1001 0000 0100 0000
    nodeRegisterManager->nodeRegisterMap["/r2/right_leg/gripper/joint0"].status["/r2/right_leg/gripper/joint0/StatReg1"].registerValue = 0x9040; // 1001 0000 0100 0000
    nodeRegisterManager->nodeRegisterMap["/r2/left_arm/joint0"].status["/r2/left_arm/joint0/StatReg1"].registerValue                   = 0x9040; // 1001 0000 0100 0000
    nodeRegisterManager->nodeRegisterMap["/r2/neck/joint0"].status["/r2/neck/joint0/StatReg1"].registerValue                           = 0x9040; // 1001 0000 0100 0000

    jointControlFsmMap["/r2/left_leg/joint0"]->updateCommandModeState();
    jointControlFsmMap["/r2/right_leg/gripper/joint0"]->updateCommandModeState();
    jointControlFsmMap["/r2/left_arm/joint0"]->updateCommandModeState();
    jointControlFsmMap["/r2/neck/joint0"]->updateCommandModeState();

    EXPECT_EQ(nasa_r2_common_msgs::JointControlCommandMode::MULTILOOPSTEP, jointControlFsmMap["/r2/left_leg/joint0"]->getStates().commandMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlCommandMode::MULTILOOPSTEP, jointControlFsmMap["/r2/right_leg/gripper/joint0"]->getStates().commandMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlCommandMode::MULTILOOPSTEP, jointControlFsmMap["/r2/left_arm/joint0"]->getStates().commandMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlCommandMode::MULTILOOPSTEP, jointControlFsmMap["/r2/neck/joint0"]->getStates().commandMode.state);
}

TEST_F(JointControlActualFsmTest, MultiLoopSmooth)
{
    // Fake coeffs loaded and MultiLoop
    nodeRegisterManager->nodeRegisterMap["/r2/left_leg/joint0"].status["/r2/left_leg/joint0/StatReg2"].registerValue                   = 0x0103; // 0000 0001 0000 0011
    nodeRegisterManager->nodeRegisterMap["/r2/right_leg/gripper/joint0"].status["/r2/right_leg/gripper/joint0/StatReg2"].registerValue = 0x0103; // 0000 0001 0000 0011
    nodeRegisterManager->nodeRegisterMap["/r2/left_arm/joint0"].status["/r2/left_arm/joint0/StatReg2"].registerValue                   = 0x0103; // 0000 0001 0000 0011
    nodeRegisterManager->nodeRegisterMap["/r2/neck/joint0"].status["/r2/neck/joint0/StatReg2"].registerValue                           = 0x0103; // 0000 0001 0000 0011

    // Fake MotComSource == 1
    nodeRegisterManager->nodeRegisterMap["/r2/left_leg/joint0"].status["/r2/left_leg/joint0/StatReg1"].registerValue                   = 0x9040; // 1001 0000 0100 0000
    nodeRegisterManager->nodeRegisterMap["/r2/right_leg/gripper/joint0"].status["/r2/right_leg/gripper/joint0/StatReg1"].registerValue = 0x9040; // 1001 0000 0100 0000
    nodeRegisterManager->nodeRegisterMap["/r2/left_arm/joint0"].status["/r2/left_arm/joint0/StatReg1"].registerValue                   = 0x9040; // 1001 0000 0100 0000
    nodeRegisterManager->nodeRegisterMap["/r2/neck/joint0"].status["/r2/neck/joint0/StatReg1"].registerValue                           = 0x9040; // 1001 0000 0100 0000

    // Fake PosComVelocity == 1
    nodeRegisterManager->nodeRegisterMap["/r2/left_leg/joint0"].status["/r2/left_leg/joint0/StatReg3"].registerValue                   = 0x0100; // 0000 0001 0000 0000
    nodeRegisterManager->nodeRegisterMap["/r2/right_leg/gripper/joint0"].status["/r2/right_leg/gripper/joint0/StatReg3"].registerValue = 0x0100; // 0000 0001 0000 0000
    nodeRegisterManager->nodeRegisterMap["/r2/left_arm/joint0"].status["/r2/left_arm/joint0/StatReg3"].registerValue                   = 0x0100; // 0000 0001 0000 0000
    nodeRegisterManager->nodeRegisterMap["/r2/neck/joint0"].status["/r2/neck/joint0/StatReg3"].registerValue                           = 0x0100; // 0000 0001 0000 0000

    jointControlFsmMap["/r2/left_leg/joint0"]->updateCommandModeState();
    jointControlFsmMap["/r2/right_leg/gripper/joint0"]->updateCommandModeState();
    jointControlFsmMap["/r2/left_arm/joint0"]->updateCommandModeState();
    jointControlFsmMap["/r2/neck/joint0"]->updateCommandModeState();

    EXPECT_EQ(nasa_r2_common_msgs::JointControlCommandMode::MULTILOOPSMOOTH, jointControlFsmMap["/r2/left_leg/joint0"]->getStates().commandMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlCommandMode::MULTILOOPSTEP,   jointControlFsmMap["/r2/right_leg/gripper/joint0"]->getStates().commandMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlCommandMode::MULTILOOPSMOOTH, jointControlFsmMap["/r2/left_arm/joint0"]->getStates().commandMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlCommandMode::MULTILOOPSMOOTH, jointControlFsmMap["/r2/neck/joint0"]->getStates().commandMode.state);
}

TEST_F(JointControlActualFsmTest, InitToDisableCalibration)
{
    EXPECT_EQ(nasa_r2_common_msgs::JointControlCalibrationMode::DISABLE, jointControlFsmMap["/r2/left_leg/joint0"]->getStates().calibrationMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlCalibrationMode::DISABLE, jointControlFsmMap["/r2/right_leg/gripper/joint0"]->getStates().calibrationMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlCalibrationMode::DISABLE, jointControlFsmMap["/r2/left_arm/joint0"]->getStates().calibrationMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlCalibrationMode::DISABLE, jointControlFsmMap["/r2/neck/joint0"]->getStates().calibrationMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlCalibrationMode::IGNORE,  jointControlFsmMap["/r2/right_arm/wrist"]->getStates().calibrationMode.state);
}

TEST_F(JointControlActualFsmTest, EnableCalibrationAndBack)
{
    nodeRegisterManager->nodeRegisterMap["/r2/left_leg/joint0"].status["/r2/left_leg/joint0/StatReg2"].registerValue                   = 0x8000; // 1000 0000 0000 0000
    nodeRegisterManager->nodeRegisterMap["/r2/right_leg/gripper/joint0"].status["/r2/right_leg/gripper/joint0/StatReg2"].registerValue = 0x8000; // 1000 0000 0000 0000
    nodeRegisterManager->nodeRegisterMap["/r2/left_arm/joint0"].status["/r2/left_arm/joint0/StatReg2"].registerValue                   = 0x8000; // 1000 0000 0000 0000
    nodeRegisterManager->nodeRegisterMap["/r2/neck/joint0"].status["/r2/neck/joint0/StatReg2"].registerValue                           = 0x8000; // 1000 0000 0000 0000

    jointControlFsmMap["/r2/left_leg/joint0"]->updateCalibrationModeState();
    jointControlFsmMap["/r2/right_leg/gripper/joint0"]->updateCalibrationModeState();
    jointControlFsmMap["/r2/left_arm/joint0"]->updateCalibrationModeState();
    jointControlFsmMap["/r2/neck/joint0"]->updateCalibrationModeState();

    EXPECT_EQ(nasa_r2_common_msgs::JointControlCalibrationMode::ENABLE, jointControlFsmMap["/r2/left_leg/joint0"]->getStates().calibrationMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlCalibrationMode::ENABLE, jointControlFsmMap["/r2/right_leg/gripper/joint0"]->getStates().calibrationMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlCalibrationMode::ENABLE, jointControlFsmMap["/r2/left_arm/joint0"]->getStates().calibrationMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlCalibrationMode::ENABLE, jointControlFsmMap["/r2/neck/joint0"]->getStates().calibrationMode.state);

    nodeRegisterManager->nodeRegisterMap["/r2/left_leg/joint0"].status["/r2/left_leg/joint0/StatReg2"].registerValue                   = 0x0000; // 0000 0000 0000 0000
    nodeRegisterManager->nodeRegisterMap["/r2/right_leg/gripper/joint0"].status["/r2/right_leg/gripper/joint0/StatReg2"].registerValue = 0x0000; // 0000 0000 0000 0000
    nodeRegisterManager->nodeRegisterMap["/r2/left_arm/joint0"].status["/r2/left_arm/joint0/StatReg2"].registerValue                   = 0x0000; // 0000 0000 0000 0000
    nodeRegisterManager->nodeRegisterMap["/r2/neck/joint0"].status["/r2/neck/joint0/StatReg2"].registerValue                           = 0x0000; // 0000 0000 0000 0000

    jointControlFsmMap["/r2/left_leg/joint0"]->updateCalibrationModeState();
    jointControlFsmMap["/r2/right_leg/gripper/joint0"]->updateCalibrationModeState();
    jointControlFsmMap["/r2/left_arm/joint0"]->updateCalibrationModeState();
    jointControlFsmMap["/r2/neck/joint0"]->updateCalibrationModeState();

    EXPECT_EQ(nasa_r2_common_msgs::JointControlCalibrationMode::DISABLE, jointControlFsmMap["/r2/left_leg/joint0"]->getStates().calibrationMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlCalibrationMode::DISABLE, jointControlFsmMap["/r2/right_leg/gripper/joint0"]->getStates().calibrationMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlCalibrationMode::DISABLE, jointControlFsmMap["/r2/left_arm/joint0"]->getStates().calibrationMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlCalibrationMode::DISABLE, jointControlFsmMap["/r2/neck/joint0"]->getStates().calibrationMode.state);
}

TEST_F(JointControlActualFsmTest, InitToDisableClearFault)
{
    EXPECT_EQ(nasa_r2_common_msgs::JointControlClearFaultMode::DISABLE, jointControlFsmMap["/r2/left_leg/joint0"]->getStates().clearFaultMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlClearFaultMode::DISABLE, jointControlFsmMap["/r2/right_leg/gripper/joint0"]->getStates().clearFaultMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlClearFaultMode::DISABLE, jointControlFsmMap["/r2/left_arm/joint0"]->getStates().clearFaultMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlClearFaultMode::DISABLE, jointControlFsmMap["/r2/neck/joint0"]->getStates().clearFaultMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlClearFaultMode::DISABLE, jointControlFsmMap["/r2/right_arm/wrist"]->getStates().clearFaultMode.state);
}

TEST_F(JointControlActualFsmTest, EnableClearFaultAndBack)
{
    // Fake clear fault ack
    nodeRegisterManager->nodeRegisterMap["/r2/left_leg/joint0"].status["/r2/left_leg/joint0/StatReg2"].registerValue                   = 0x0040; // 0000 0000 0100 0000
    nodeRegisterManager->nodeRegisterMap["/r2/right_leg/gripper/joint0"].status["/r2/right_leg/gripper/joint0/StatReg2"].registerValue = 0x0040; // 0000 0000 0100 0000
    nodeRegisterManager->nodeRegisterMap["/r2/left_arm/joint0"].status["/r2/left_arm/joint0/StatReg2"].registerValue                   = 0x0040; // 0000 0000 0100 0000
    nodeRegisterManager->nodeRegisterMap["/r2/neck/joint0"].status["/r2/neck/joint0/StatReg2"].registerValue                           = 0x0040; // 0000 0000 0100 0000

    EXPECT_EQ(nasa_r2_common_msgs::JointControlClearFaultMode::ENABLE, jointControlFsmMap["/r2/left_leg/joint0"]->getStates().clearFaultMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlClearFaultMode::ENABLE, jointControlFsmMap["/r2/right_leg/gripper/joint0"]->getStates().clearFaultMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlClearFaultMode::ENABLE, jointControlFsmMap["/r2/neck/joint0"]->getStates().clearFaultMode.state);

    // Unfake clear fault ack
    nodeRegisterManager->nodeRegisterMap["/r2/left_leg/joint0"].status["/r2/left_leg/joint0/StatReg2"].registerValue                   = 0x0000; // 0000 0000 0000 0000
    nodeRegisterManager->nodeRegisterMap["/r2/right_leg/gripper/joint0"].status["/r2/right_leg/gripper/joint0/StatReg2"].registerValue = 0x0000; // 0000 0000 0000 0000
    nodeRegisterManager->nodeRegisterMap["/r2/left_arm/joint0"].status["/r2/left_arm/joint0/StatReg2"].registerValue                   = 0x0000; // 0000 0000 0000 0000
    nodeRegisterManager->nodeRegisterMap["/r2/neck/joint0"].status["/r2/neck/joint0/StatReg2"].registerValue                           = 0x0000; // 0000 0000 0000 0000

    EXPECT_EQ(nasa_r2_common_msgs::JointControlClearFaultMode::DISABLE, jointControlFsmMap["/r2/left_leg/joint0"]->getStates().clearFaultMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlClearFaultMode::DISABLE, jointControlFsmMap["/r2/right_leg/gripper/joint0"]->getStates().clearFaultMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlClearFaultMode::DISABLE, jointControlFsmMap["/r2/left_arm/joint0"]->getStates().clearFaultMode.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlClearFaultMode::DISABLE, jointControlFsmMap["/r2/neck/joint0"]->getStates().clearFaultMode.state);
}

TEST_F(JointControlActualFsmTest, InitToCoeffsNotLoaded)
{
    EXPECT_EQ(nasa_r2_common_msgs::JointControlCoeffState::NOTLOADED, jointControlFsmMap["/r2/left_leg/joint0"]->getStates().coeffState.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlCoeffState::NOTLOADED, jointControlFsmMap["/r2/right_leg/gripper/joint0"]->getStates().coeffState.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlCoeffState::NOTLOADED, jointControlFsmMap["/r2/left_arm/joint0"]->getStates().coeffState.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlCoeffState::NOTLOADED, jointControlFsmMap["/r2/neck/joint0"]->getStates().coeffState.state);
}

TEST_F(JointControlActualFsmTest, ShowCoeffsLoadedAndBack)
{
    nodeRegisterManager->nodeRegisterMap["/r2/left_leg/joint0"].status["/r2/left_leg/joint0/StatReg2"].registerValue                   = 0x0100; // 0000 0001 0000 0000
    nodeRegisterManager->nodeRegisterMap["/r2/right_leg/gripper/joint0"].status["/r2/right_leg/gripper/joint0/StatReg2"].registerValue = 0x0100; // 0000 0001 0000 0000
    nodeRegisterManager->nodeRegisterMap["/r2/left_arm/joint0"].status["/r2/left_arm/joint0/StatReg2"].registerValue                   = 0x0100; // 0000 0001 0000 0000
    nodeRegisterManager->nodeRegisterMap["/r2/neck/joint0"].status["/r2/neck/joint0/StatReg2"].registerValue                           = 0x0100; // 0000 0001 0000 0000

    jointControlFsmMap["/r2/left_leg/joint0"]->updateCoeffState();
    jointControlFsmMap["/r2/right_leg/gripper/joint0"]->updateCoeffState();
    jointControlFsmMap["/r2/left_arm/joint0"]->updateCoeffState();
    jointControlFsmMap["/r2/neck/joint0"]->updateCoeffState();

    EXPECT_EQ(nasa_r2_common_msgs::JointControlCoeffState::LOADED, jointControlFsmMap["/r2/left_leg/joint0"]->getStates().coeffState.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlCoeffState::LOADED, jointControlFsmMap["/r2/right_leg/gripper/joint0"]->getStates().coeffState.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlCoeffState::LOADED, jointControlFsmMap["/r2/left_arm/joint0"]->getStates().coeffState.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlCoeffState::LOADED, jointControlFsmMap["/r2/neck/joint0"]->getStates().coeffState.state);

    nodeRegisterManager->nodeRegisterMap["/r2/left_leg/joint0"].status["/r2/left_leg/joint0/StatReg2"].registerValue                   = 0x0000; // 0000 0000 0000 0000
    nodeRegisterManager->nodeRegisterMap["/r2/right_leg/gripper/joint0"].status["/r2/right_leg/gripper/joint0/StatReg2"].registerValue = 0x0000; // 0000 0000 0000 0000
    nodeRegisterManager->nodeRegisterMap["/r2/left_arm/joint0"].status["/r2/left_arm/joint0/StatReg2"].registerValue                   = 0x0000; // 0000 0000 0000 0000
    nodeRegisterManager->nodeRegisterMap["/r2/neck/joint0"].status["/r2/neck/joint0/StatReg2"].registerValue                           = 0x0000; // 0000 0000 0000 0000

    jointControlFsmMap["/r2/left_leg/joint0"]->updateCoeffState();
    jointControlFsmMap["/r2/right_leg/gripper/joint0"]->updateCoeffState();
    jointControlFsmMap["/r2/left_arm/joint0"]->updateCoeffState();
    jointControlFsmMap["/r2/neck/joint0"]->updateCoeffState();

    EXPECT_EQ(nasa_r2_common_msgs::JointControlCoeffState::NOTLOADED, jointControlFsmMap["/r2/left_leg/joint0"]->getStates().coeffState.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlCoeffState::NOTLOADED, jointControlFsmMap["/r2/right_leg/gripper/joint0"]->getStates().coeffState.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlCoeffState::NOTLOADED, jointControlFsmMap["/r2/left_arm/joint0"]->getStates().coeffState.state);
    EXPECT_EQ(nasa_r2_common_msgs::JointControlCoeffState::NOTLOADED, jointControlFsmMap["/r2/neck/joint0"]->getStates().coeffState.state);
}

int main(int argc, char **argv){
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
