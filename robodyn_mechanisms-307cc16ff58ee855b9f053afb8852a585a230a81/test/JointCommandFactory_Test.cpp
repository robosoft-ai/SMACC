#include <gtest/gtest.h>
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include "nasa_r2_config_core/RobotInstanceFactory.h"
#include "nasa_r2_config_core/CoeffMapLoader.h"
#include "nasa_robodyn_mechanisms_core/JointCommandFactory.h"
#include <ros/package.h>

using namespace std;

std::map<std::string, float>    testFloats;
std::map<std::string, int16_t>  testInt16s;
std::map<std::string, uint16_t> testUInt16s;
std::map<std::string, int32_t>  testInt32s;
std::map<std::string, uint32_t> testUInt32s;
std::map<std::string, float>    testLiveCoeffs;
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

float getInt16(std::string item)
{
    return testInt16s[item];
}

float getUInt16(std::string item)
{
    return testUInt16s[item];
}

void setUInt16(std::string item, uint16_t value)
{
    testUInt16s[item] = value;
}

uint32_t getInt32(std::string item)
{
    return testInt32s[item];
}

uint32_t getUInt32(std::string item)
{
    return testUInt32s[item];
}

void setInt16(std::string item, int16_t value)
{
    testInt16s[item] = value;
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

class JointCommandFactoryTest : public ::testing::Test {
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

            testFloats.clear();

            io.getFloat          = getFloat;
            io.setFloat          = setFloat;
            io.setUInt16         = setUInt16;
            io.getUInt16         = getUInt16;
            io.getInt16          = getInt16;
            io.setInt16          = setInt16;
            io.getInt32          = getInt32;
            io.getUInt32         = getUInt32;
            io.getMotorCoeff     = getMotorCoeff;
            io.hasBrainstemCoeff = hasBrainstemCoeff;
            io.getBrainstemCoeff = getBrainstemCoeff;
            io.hasLiveCoeff      = hasLiveCoeff;
            io.getLiveCoeff      = getLiveCoeff;
            io.setLiveCoeff      = setLiveCoeff;
            io.getJointNames     = getJointNames;
            io.getActuatorNames  = getActuatorNames;
            io.getCommandFile    = getCommandFile;

            ros::Time::init();
        }

        virtual void TearDown()
        {
        }

        string packagePath, configPath, configSafetyPath, fileName;
        JointCommandInterface::IoFunctions io;
        vector<JointCommandInterfacePtr> jointInterfacePtrs;
};

TEST_F(JointCommandFactoryTest, generate)
{
    for (unsigned int i = 0; i < instance->mechanisms.size(); ++i)
    {
        std::cout<<"Loading mechanism: " << instance->mechanisms[i] <<std::endl;
        jointInterfacePtrs.push_back(JointCommandFactory::generate(instance->mechanisms[i], io));

        std::cout << jointInterfacePtrs[i].get() << std::endl;

        //! @todo this will work when Fingers and Wrist interface classes are implemented
        // EXPECT_TRUE(jointInterfacePtrs[i].get());
    }

    EXPECT_EQ(instance->mechanisms.size(), jointInterfacePtrs.size());
}

int main(int argc, char **argv){
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
