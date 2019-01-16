#include <gtest/gtest.h>
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include "nasa_robodyn_mechanisms_core/TubeTareMechanism.h"
#include "nasa_r2_config_core/RobotInstanceFactory.h"
#include "nasa_r2_config_core/CoeffMapLoader.h"
#include "ros/package.h"
#include "ros/time.h"


using namespace std;

std::map<std::string, float> testFloats;
std::map<std::string, int16_t> testInt16;
std::map<std::string, uint16_t> testUint16;
CoeffMapPtr motorCoeffMap;
CoeffMapPtr brainstemCoeffMap;
std::map<std::string, float> testLiveCoeffs;
RobotInstancePtr instance;


float getFloat(std::string item)
{
    return testFloats[item];
}

float getInt16(std::string item)
{
    return testInt16[item];
}

float getUint16(std::string item)
{
    return testUint16[item];
}

void setUint16(std::string item, uint16_t value)
{
    testUint16[item] = value;
}

void setInt16(std::string item, int16_t value)
{
    testInt16[item] = value;
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

class TubeTareMechanismTest : public ::testing::Test {
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

            // Set up function pointers
            testFloats.clear();
            testInt16.clear();
            testUint16.clear();
            testLiveCoeffs.clear();

            io.getFloat         = getFloat;
            io.setInt16         = setInt16;
            io.getInt16         = getInt16;
            io.getUInt16        = getUint16;
            io.setUInt16        = setUint16;
            io.getMotorCoeff         = getMotorCoeff;
            io.hasLiveCoeff   = hasLiveCoeff;
            io.getLiveCoeff   = getLiveCoeff;
            io.setLiveCoeff   = setLiveCoeff;
            io.getJointNames    = getJointNames;
            io.getActuatorNames = getActuatorNames;
            io.getCommandFile   = getCommandFile;

            ros::Time::init();
        }

        virtual void TearDown()
        {
        }

        string packagePath, configPath, configSafetyPath, fileName;
        TubeTareInterface::IoFunctions io;
};

TEST_F(TubeTareMechanismTest, Constructor)
{
    ASSERT_NO_THROW(TubeTareMechanism<4> thumbJoint("/r2/right_arm/hand/thumb", io, 500));
    ASSERT_NO_THROW(TubeTareMechanism<3> fingerJoint("/r2/right_arm/hand/index", io, 500));
    ASSERT_NO_THROW(TubeTareMechanism<2> ringlittleJoint("/r2/right_arm/hand/ringlittle", io, 500));

//    //! Missing IO function definitions
    io.getInt16 = 0;
    ASSERT_THROW(TubeTareMechanism<4> thumbJoint("/r2/right_arm/hand/thumb", io, 500), std::invalid_argument);
    ASSERT_THROW(TubeTareMechanism<3> fingerJoint("/r2/right_arm/hand/index", io, 500), std::invalid_argument);
    ASSERT_THROW(TubeTareMechanism<2> ringlittleJoint("/r2/right_arm/hand/ringlittle", io, 500), std::invalid_argument);
}

TEST_F(TubeTareMechanismTest, setStop)
{
    TubeTareMechanism<4> thumbJoint("/r2/right_arm/hand/thumb", io, 500);

    setInt16("/r2/right_arm/hand/thumb/proximal_extensor/MotCom", 0);
    setInt16("/r2/right_arm/hand/thumb/medial_extensor/MotCom", 1);
    setInt16("/r2/right_arm/hand/thumb/medial_flexor/MotCom", 2);
    setInt16("/r2/right_arm/hand/thumb/distal_extensor/MotCom", 3);
    setInt16("/r2/right_arm/hand/thumb/distal_flexor/MotCom", 4);

    thumbJoint.setStop(std::vector<std::string>());

    EXPECT_FLOAT_EQ(0, getInt16("/r2/right_arm/hand/thumb/proximal_extensor/MotCom"));
    EXPECT_FLOAT_EQ(0, getInt16("/r2/right_arm/hand/thumb/medial_extensor/MotCom"));
    EXPECT_FLOAT_EQ(0, getInt16("/r2/right_arm/hand/thumb/medial_flexor/MotCom"));
    EXPECT_FLOAT_EQ(0, getInt16("/r2/right_arm/hand/thumb/distal_extensor/MotCom"));
    EXPECT_FLOAT_EQ(0, getInt16("/r2/right_arm/hand/thumb/distal_flexor/MotCom"));

    TubeTareMechanism<3> fingerJoint("/r2/right_arm/hand/index", io, 500);

    setInt16("/r2/right_arm/hand/index/proximal_extensor/MotCom", 0);
    setInt16("/r2/right_arm/hand/index/proximal_flexor/MotCom", 1);
    setInt16("/r2/right_arm/hand/index/medial_extensor/MotCom", 2);
    setInt16("/r2/right_arm/hand/index/medial_flexor/MotCom", 3);

    fingerJoint.setStop(std::vector<std::string>());

    EXPECT_FLOAT_EQ(0, getInt16("/r2/right_arm/hand/index/proximal_extensor/MotCom"));
    EXPECT_FLOAT_EQ(0, getInt16("/r2/right_arm/hand/index/proximal_flexor/MotCom"));
    EXPECT_FLOAT_EQ(0, getInt16("/r2/right_arm/hand/index/medial_extensor/MotCom"));
    EXPECT_FLOAT_EQ(0, getInt16("/r2/right_arm/hand/index/medial_flexor/MotCom"));

    TubeTareMechanism<2> ringlittleJoint("/r2/right_arm/hand/ringlittle", io, 500);

    setInt16("/r2/right_arm/hand/ringlittle/ringlittle_extensor/MotCom", 0);
    setInt16("/r2/right_arm/hand/ringlittle/ring_flexor/MotCom", 1);
    setInt16("/r2/right_arm/hand/ringlittle/little_flexor/MotCom", 2);

    ringlittleJoint.setStop(std::vector<std::string>());

    EXPECT_FLOAT_EQ(0, getInt16("/r2/right_arm/hand/ringlittle/ringlittle_extensor/MotCom"));
    EXPECT_FLOAT_EQ(0, getInt16("/r2/right_arm/hand/ringlittle/ring_flexor/MotCom"));
    EXPECT_FLOAT_EQ(0, getInt16("/r2/right_arm/hand/ringlittle/little_flexor/MotCom"));

}

TEST_F(TubeTareMechanismTest, isMoving)
{
    TubeTareMechanism<4> thumbJoint("/r2/right_arm/hand/thumb", io, 500);

    for(int i = 0; i<20; ++i)
    {
        setInt16("/r2/right_arm/hand/thumb/proximal_extensor/IncEnc", i);
        bool moving = thumbJoint.isMoving();
        if(i>3)
            EXPECT_TRUE(moving);
    }

    for(int i = 0; i >-20; --i)
    {
        setInt16("/r2/right_arm/hand/thumb/proximal_extensor/IncEnc", i);
        bool moving = thumbJoint.isMoving();
        if(i>3)
            EXPECT_TRUE(moving);
    }

}

TEST_F(TubeTareMechanismTest, setRelease)
{
    TubeTareMechanism<3> fingerJoint("/r2/right_arm/hand/index", io, 500);

    setInt16("/r2/right_arm/hand/index/proximal_extensor/MotCom", 10);
    setInt16("/r2/right_arm/hand/index/proximal_flexor/MotCom", 20);
    setInt16("/r2/right_arm/hand/index/medial_extensor/MotCom", 30);
    setInt16("/r2/right_arm/hand/index/medial_flexor/MotCom", 40);

    float releaseVal = io.getMotorCoeff("/r2/right_arm/hand/index/MotComReleaseProximalExt");

    fingerJoint.setRelease(std::vector<std::string>());

    EXPECT_FLOAT_EQ(releaseVal, getInt16("/r2/right_arm/hand/index/proximal_extensor/MotCom"));
}

TEST_F(TubeTareMechanismTest, getEncTarePos)
{

    TubeTareMechanism<2> ringlittleJoint("/r2/right_arm/hand/ringlittle", io, 500);

    setInt16("/r2/right_arm/hand/ringlittle/ringlittle_extensor/IncEnc", 0);
    setInt16("/r2/right_arm/hand/ringlittle/ring_flexor/IncEnc", 1);
    setInt16("/r2/right_arm/hand/ringlittle/little_flexor/IncEnc", 2);

    ringlittleJoint.getEncoderTarePos();

    EXPECT_FLOAT_EQ(0, getLiveCoeff("/r2/right_arm/hand/ringlittle/ringlittle_extensor/EncoderOffset"));
    EXPECT_FLOAT_EQ(1, getLiveCoeff("/r2/right_arm/hand/ringlittle/ring_flexor/EncoderOffset"));
    EXPECT_FLOAT_EQ(2, getLiveCoeff("/r2/right_arm/hand/ringlittle/little_flexor/EncoderOffset"));
}

TEST_F(TubeTareMechanismTest, setTighten)
{
    TubeTareMechanism<4> thumbJoint("/r2/right_arm/hand/thumb", io, 500);

    setInt16("/r2/right_arm/hand/thumb/proximal_extensor/MotCom", 10);

    float tightenVal = io.getMotorCoeff("/r2/right_arm/hand/thumb/MotComTightenMainExt");

    thumbJoint.setTighten(std::vector<std::string>());

    EXPECT_FLOAT_EQ(tightenVal, getInt16("/r2/right_arm/hand/thumb/proximal_extensor/MotCom"));

}

TEST_F(TubeTareMechanismTest, goodPosition)
{
    TubeTareMechanism<3> fingerJoint("/r2/right_arm/hand/index", io, 500);

    float minTravel = io.getMotorCoeff("/r2/right_arm/hand/index/EncoderMinTravel");
    float maxTravel = io.getMotorCoeff("/r2/right_arm/hand/index/EncoderMaxTravel");

    setInt16("/r2/right_arm/hand/index/proximal_extensor/IncEnc", minTravel-1);
    setInt16("/r2/right_arm/hand/index/proximal_flexor/IncEnc", minTravel-1);
    setInt16("/r2/right_arm/hand/index/medial_extensor/IncEnc", minTravel-1);
    setInt16("/r2/right_arm/hand/index/medial_flexor/IncEnc", minTravel-1);

    EXPECT_FLOAT_EQ(-1, fingerJoint.goodPosition());

    setInt16("/r2/right_arm/hand/index/proximal_extensor/IncEnc", maxTravel-1);
    setInt16("/r2/right_arm/hand/index/proximal_flexor/IncEnc", maxTravel-1);
    setInt16("/r2/right_arm/hand/index/medial_extensor/IncEnc", maxTravel-1);
    setInt16("/r2/right_arm/hand/index/medial_flexor/IncEnc", maxTravel-1);

    EXPECT_FLOAT_EQ(0, fingerJoint.goodPosition());

    setInt16("/r2/right_arm/hand/index/proximal_extensor/IncEnc", maxTravel+1);
    setInt16("/r2/right_arm/hand/index/proximal_flexor/IncEnc", maxTravel+1);
    setInt16("/r2/right_arm/hand/index/medial_extensor/IncEnc", maxTravel+1);
    setInt16("/r2/right_arm/hand/index/medial_flexor/IncEnc", maxTravel+1);

    EXPECT_FLOAT_EQ(1, fingerJoint.goodPosition());

}

TEST_F(TubeTareMechanismTest, getSliderTarePos)
{
    TubeTareMechanism<2> ringlittleJoint("/r2/right_arm/hand/ringlittle", io, 500);

    setInt16("/r2/right_arm/hand/ringlittle/ringlittle_extensor/IncEnc", 0);
    setInt16("/r2/right_arm/hand/ringlittle/ring_flexor/IncEnc", 1);
    setInt16("/r2/right_arm/hand/ringlittle/little_flexor/IncEnc", 2);

    setUint16("/r2/right_arm/hand/ringlittle/ring/Hall", 30);
    setUint16("/r2/right_arm/hand/ringlittle/little/Hall", 30);

    ringlittleJoint.getSliderTarePos();

    EXPECT_FLOAT_EQ(0, getLiveCoeff("/r2/right_arm/hand/ringlittle/ringlittle_extensor/SliderOffset"));
    EXPECT_FLOAT_EQ(1, getLiveCoeff("/r2/right_arm/hand/ringlittle/ring_flexor/SliderOffset"));
    EXPECT_FLOAT_EQ(2, getLiveCoeff("/r2/right_arm/hand/ringlittle/little_flexor/SliderOffset"));

//    EXPECT_FLOAT_EQ(0, getLiveCoeff("/r2/right_arm/hand/ringlittle/ringlittle_extensor/SliderPosition"));
//    EXPECT_FLOAT_EQ(1, getLiveCoeff("/r2/right_arm/hand/ringlittle/ring_flexor/SliderPosition"));
//    EXPECT_FLOAT_EQ(2, getLiveCoeff("/r2/right_arm/hand/ringlittle/little_flexor/SliderPosition"));

}

TEST_F(TubeTareMechanismTest, getTensionTareValue)
{
    TubeTareMechanism<3> fingerJoint("/r2/right_arm/hand/index", io, 500);

    setUint16("/r2/right_arm/hand/index/proximal_extensor/TensionA", 0);
    setUint16("/r2/right_arm/hand/index/proximal_flexor/TensionA", 1);
    setUint16("/r2/right_arm/hand/index/medial_extensor/TensionA", 2);
    setUint16("/r2/right_arm/hand/index/medial_flexor/TensionA", 3);
    setUint16("/r2/right_arm/hand/index/proximal_extensor/TensionB", 0);
    setUint16("/r2/right_arm/hand/index/proximal_flexor/TensionB", 1);
    setUint16("/r2/right_arm/hand/index/medial_extensor/TensionB", 2);
    setUint16("/r2/right_arm/hand/index/medial_flexor/TensionB", 3);
    
    fingerJoint.getTensionTareValue();
    
    EXPECT_FLOAT_EQ(650, getLiveCoeff("/r2/right_arm/hand/index/proximal_extensor/TensionOffset"));
    EXPECT_FLOAT_EQ(650.02, getLiveCoeff("/r2/right_arm/hand/index/proximal_flexor/TensionOffset"));
    EXPECT_FLOAT_EQ(650.04, getLiveCoeff("/r2/right_arm/hand/index/medial_extensor/TensionOffset"));
    EXPECT_FLOAT_EQ(650.06, getLiveCoeff("/r2/right_arm/hand/index/medial_flexor/TensionOffset"));
    
    setUint16("/r2/right_arm/hand/index/proximal_extensor/TensionA", 40);
    setUint16("/r2/right_arm/hand/index/proximal_flexor/TensionA", 10);
    setUint16("/r2/right_arm/hand/index/medial_extensor/TensionA", 20);
    setUint16("/r2/right_arm/hand/index/medial_flexor/TensionA", 30);
    setUint16("/r2/right_arm/hand/index/proximal_extensor/TensionB", 40);
    setUint16("/r2/right_arm/hand/index/proximal_flexor/TensionB", 10);
    setUint16("/r2/right_arm/hand/index/medial_extensor/TensionB", 20);
    setUint16("/r2/right_arm/hand/index/medial_flexor/TensionB", 30);
    
    fingerJoint.getTensionTareValue();
    
    EXPECT_FLOAT_EQ(650.4, getLiveCoeff("/r2/right_arm/hand/index/proximal_extensor/TensionOffset"));
    EXPECT_FLOAT_EQ(650.11, getLiveCoeff("/r2/right_arm/hand/index/proximal_flexor/TensionOffset"));
    EXPECT_FLOAT_EQ(650.22, getLiveCoeff("/r2/right_arm/hand/index/medial_extensor/TensionOffset"));
    EXPECT_FLOAT_EQ(650.33, getLiveCoeff("/r2/right_arm/hand/index/medial_flexor/TensionOffset"));
}

int main(int argc, char **argv){
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
