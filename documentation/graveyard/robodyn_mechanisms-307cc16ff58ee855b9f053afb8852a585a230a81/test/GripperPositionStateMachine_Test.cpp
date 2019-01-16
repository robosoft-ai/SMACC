#include <gtest/gtest.h>
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include "nasa_robodyn_mechanisms_core/GripperPositionStateMachine.h"

using namespace std;

namespace sc = boost::statechart;
namespace mpl = boost::mpl;
namespace scu = StatechartUtilities;

std::map<std::string, uint16_t> testUInt16s;

uint16_t getUInt16(const std::string& name)
{
    return testUInt16s[name];
}

void setUInt16(const std::string& name, uint16_t value)
{
    testUInt16s[name] = value;
}

std::map<std::string, float> testFloats;

float getFloat(const std::string& name)
{
    return testFloats[name];
}

void setFloat(const std::string& name, float value)
{
    testFloats[name] = value;
}

void sendCommand(const nasa_r2_common_msgs::JointCommand& msg)
{
    //cout << msg << endl;
}

class GripperPositionStateMachineTest : public ::testing::Test {
    protected:
        virtual void SetUp()
        {
            gp = boost::make_shared<GripperPosition>();
            gp->mechanism = "/r2/left_leg/gripper";
            gp->getFloat = getFloat;
            gp->setFloat = setFloat;
            gp->getUInt16 = getUInt16;
            gp->setUInt16 = setUInt16;
            gp->dutyCycleLimitName = "/r2/left_leg/gripper/joint0/MotComLimit";
            gp->measuredCurrentName = "/r2/left_leg/gripper/joint0/CurrentMeasured";
            gp->currentLimitName = "/r2/left_leg/gripper/joint0/VLCurLimit";
            gp->jointCommand.name.assign(1, "/r2/left_leg/gripper/joint0");
            gp->sendCommand = sendCommand;
            gp->jawOpenPosition = 0.5;
            gp->encoderOpenVelocity = 15;
            gp->encoderCloseVelocity = -15;
            gp->encoderClosedPosition = 0;
            gp->encoderState.position.assign(1, 0);
            gp->jawLeftState.position.assign(1, 0);
            gp->jawRightState.position.assign(1, 0);
            gp->encoderState.velocity.assign(1, 0);
            gp->encoderState.effort.assign(1, 0);
            gp->jawLeftState.effort.assign(1, 0);
            gp->jawRightState.effort.assign(1, 0);
            gp->jawPositionNoise = 0.005;
            gp->encoderPositionNoise = 0.1;
            gp->encoderVelocityNoise = 0.1;
            gp->currentNoise = 0.1;
            gp->currentMax = 4.0;
            gp->dutyCycleLimit = 255;
            gp->weakForce = 30;
            gp->strongForce = 30;
            gp->jawPositionForceThreshold = 0.0;
            gp->dutyCycleLimitFilter.setRates(1, 255);
            gp->outputCurrentFilter.setRates(0.05, 1000);
            gp->initiate();
        }

        virtual void TearDown()
        {
        }

        boost::shared_ptr<GripperPosition> gp;
        std::vector<std::string> states;
};

TEST_F(GripperPositionStateMachineTest, InitialStates)
{
    states = scu::getCurrentStates(gp);
    EXPECT_EQ(12, states.size());

    EXPECT_TRUE(scu::isActiveState(gp, "Container"));
    EXPECT_TRUE(scu::isActiveState(gp, "Inactive"));
    EXPECT_TRUE(scu::isActiveState(gp, "Unready"));
    EXPECT_TRUE(scu::isActiveState(gp, "NotFaulted"));
    EXPECT_TRUE(scu::isActiveState(gp, "Unloaded"));
    EXPECT_TRUE(scu::isActiveState(gp, "Locked"));
    EXPECT_TRUE(scu::isActiveState(gp, "LockedStatus"));
    EXPECT_TRUE(scu::isActiveState(gp, "LessThanDesiredJawPosition"));
    EXPECT_TRUE(scu::isActiveState(gp, "NotStalled"));
    EXPECT_TRUE(scu::isActiveState(gp, "Still"));
    EXPECT_TRUE(scu::isActiveState(gp, "NotOpen"));
    EXPECT_TRUE(scu::isActiveState(gp, "LessThanForceThreshold"));

    EXPECT_STREQ("/r2/left_leg/gripper/joint0", gp->jointCommand.name[0].c_str());
}

TEST_F(GripperPositionStateMachineTest, Loaded)
{
    //! event driven
    EXPECT_TRUE(scu::isActiveState(gp, "Unloaded"));
    gp->process_event(e_loaded());
    EXPECT_TRUE(scu::isActiveState(gp, "Loaded"));
    gp->process_event(e_unloaded());
    EXPECT_TRUE(scu::isActiveState(gp, "Unloaded"));

    //! data driven
    gp->setPoint.expectedLoad = 10;
    gp->jawLeftState.effort.assign(1, 10);
    gp->jawRightState.effort.assign(1, 10);
    gp->process_event(e_do());
    EXPECT_TRUE(scu::isActiveState(gp, "Loaded"));
    gp->jawLeftState.effort.assign(1, 9);
    gp->jawRightState.effort.assign(1, 9);
    gp->process_event(e_do());
    EXPECT_TRUE(scu::isActiveState(gp, "Unloaded"));
    gp->jawLeftState.effort.assign(1, 9);
    gp->jawRightState.effort.assign(1, 11);
    gp->process_event(e_do());
    EXPECT_TRUE(scu::isActiveState(gp, "Loaded"));    
}

TEST_F(GripperPositionStateMachineTest, Locked)
{
    //! event driven
    EXPECT_TRUE(scu::isActiveState(gp, "Locked"));
    gp->process_event(e_unlocked());
    EXPECT_TRUE(scu::isActiveState(gp, "Unlocked"));
    gp->process_event(e_locked());
    EXPECT_TRUE(scu::isActiveState(gp, "Locked"));

    //! data driven
    //! @todo this is a very hard test because the gripper model is inaccurate
    // double baseEncoderAngle = GripperKinematics::getEncoderAngle(0);
    // gp->encoderState.position.assign(1, baseEncoderAngle-2);
    // gp->jawLeftState.position.assign(1, 0);
    // gp->jawRightState.position.assign(1, 0);
    // gp->process_event(e_do());
    // EXPECT_TRUE(scu::isActiveState(gp, "Locked"));
    // gp->encoderState.position.assign(1, baseEncoderAngle+5);
    // gp->process_event(e_do());
    // EXPECT_TRUE(scu::isActiveState(gp, "Unlocked"));
}

TEST_F(GripperPositionStateMachineTest, LockedStatus)
{
    //! event driven
    EXPECT_TRUE(scu::isActiveState(gp, "LockedStatus"));
    gp->process_event(e_unlockedStatus());
    EXPECT_TRUE(scu::isActiveState(gp, "UnlockedStatus"));
    gp->process_event(e_lockedStatus());
    EXPECT_TRUE(scu::isActiveState(gp, "LockedStatus"));

    //! data driven
    //! @todo this is a very hard test because the gripper model is inaccurate
    // double baseEncoderAngle = GripperKinematics::getEncoderAngle(0);
    // gp->encoderState.position.assign(1, baseEncoderAngle-2);
    // gp->jawLeftState.position.assign(1, 0);
    // gp->jawRightState.position.assign(1, 0);
    // gp->process_event(e_do());
    // EXPECT_TRUE(scu::isActiveState(gp, "Locked"));
    // gp->encoderState.position.assign(1, baseEncoderAngle+5);
    // gp->process_event(e_do());
    // EXPECT_TRUE(scu::isActiveState(gp, "Unlocked"));
}

TEST_F(GripperPositionStateMachineTest, Ready)
{
    //! event driven
    EXPECT_TRUE(scu::isActiveState(gp, "Unready"));
    gp->process_event(e_ready());
    EXPECT_TRUE(scu::isActiveState(gp, "Ready"));
    gp->process_event(e_unready());
    EXPECT_TRUE(scu::isActiveState(gp, "Unready"));

    //! data driven
    gp->jointControlData.controlMode.state = nasa_r2_common_msgs::JointControlMode::DRIVE;
    gp->process_event(e_do());
    EXPECT_TRUE(scu::isActiveState(gp, "Ready"));
    gp->jointControlData.controlMode.state = nasa_r2_common_msgs::JointControlMode::IGNORE;
    gp->process_event(e_do());
    EXPECT_TRUE(scu::isActiveState(gp, "Unready"));
}

TEST_F(GripperPositionStateMachineTest, DesiredJawPosition)
{
    //! event driven
    EXPECT_TRUE(scu::isActiveState(gp, "LessThanDesiredJawPosition"));
    gp->process_event(e_atDesiredJawPosition());
    EXPECT_TRUE(scu::isActiveState(gp, "AtDesiredJawPosition"));
    gp->process_event(e_lessThanDesiredJawPosition());
    EXPECT_TRUE(scu::isActiveState(gp, "LessThanDesiredJawPosition"));
    gp->process_event(e_moreThanDesiredJawPosition());
    EXPECT_TRUE(scu::isActiveState(gp, "MoreThanDesiredJawPosition"));
    gp->process_event(e_atDesiredJawPosition());
    EXPECT_TRUE(scu::isActiveState(gp, "AtDesiredJawPosition"));
    gp->process_event(e_moreThanDesiredJawPosition());
    EXPECT_TRUE(scu::isActiveState(gp, "MoreThanDesiredJawPosition"));
    gp->process_event(e_lessThanDesiredJawPosition());
    EXPECT_TRUE(scu::isActiveState(gp, "LessThanDesiredJawPosition"));

    //! data driven
    gp->setPoint.jawPosition = 1;
    gp->setPoint.jawPositionDelta = 0.100001;
    gp->jawLeftState.position.assign(1, 1);
    gp->jawRightState.position.assign(1, 1);
    gp->process_event(e_do());
    EXPECT_TRUE(scu::isActiveState(gp, "AtDesiredJawPosition"));
    gp->jawLeftState.position.assign(1, 0.8);
    gp->jawRightState.position.assign(1, 0.8);
    gp->process_event(e_do());
    EXPECT_TRUE(scu::isActiveState(gp, "LessThanDesiredJawPosition"));
    gp->jawLeftState.position.assign(1, 1.2);
    gp->jawRightState.position.assign(1, 1.2);
    gp->process_event(e_do());
    EXPECT_TRUE(scu::isActiveState(gp, "MoreThanDesiredJawPosition"));
}

TEST_F(GripperPositionStateMachineTest, Stalled)
{
    //! event driven
    EXPECT_TRUE(scu::isActiveState(gp, "NotStalled"));
    gp->process_event(e_stalled());
    EXPECT_TRUE(scu::isActiveState(gp, "Stalled"));
    gp->process_event(e_notStalled());
    EXPECT_TRUE(scu::isActiveState(gp, "NotStalled"));

    //! data driven
    gp->currentLimit = 0.5;
    gp->currentNoise = 0.1;
    gp->measuredCurrent = 0.45;
    gp->process_event(e_do());
    EXPECT_TRUE(scu::isActiveState(gp, "Stalled"));
    //! can't be stalled if current limit isn't reached
    gp->measuredCurrent = 0.35;
    gp->process_event(e_do());
    EXPECT_FALSE(scu::isActiveState(gp, "Stalled"));
}

TEST_F(GripperPositionStateMachineTest, Faulted)
{
    //! event driven
    EXPECT_TRUE(scu::isActiveState(gp, "NotFaulted"));
    gp->process_event(e_faulted());
    EXPECT_TRUE(scu::isActiveState(gp, "Faulted"));
    gp->process_event(e_notFaulted());
    EXPECT_TRUE(scu::isActiveState(gp, "NotFaulted"));
}

TEST_F(GripperPositionStateMachineTest, Moving)
{
    //! event driven
    EXPECT_TRUE(scu::isActiveState(gp, "Still"));
    gp->process_event(e_moving());
    EXPECT_TRUE(scu::isActiveState(gp, "Moving"));
    gp->process_event(e_still());
    EXPECT_TRUE(scu::isActiveState(gp, "Still"));

    //! data driven
    gp->encoderState.velocity.assign(1, 42);
    gp->process_event(e_do());
    EXPECT_TRUE(scu::isActiveState(gp, "Moving"));
    gp->encoderState.velocity.assign(1, 0);
    gp->process_event(e_do());
    EXPECT_TRUE(scu::isActiveState(gp, "Still"));
}

TEST_F(GripperPositionStateMachineTest, Open)
{
    //! event driven
    EXPECT_TRUE(scu::isActiveState(gp, "NotOpen"));
    gp->process_event(e_isOpen());
    EXPECT_TRUE(scu::isActiveState(gp, "Open"));
    gp->process_event(e_isNotOpen());
    EXPECT_TRUE(scu::isActiveState(gp, "NotOpen"));

    //! data driven
    gp->jawLeftState.position.assign(1, gp->jawOpenPosition);
    gp->jawRightState.position.assign(1, gp->jawOpenPosition);
    gp->process_event(e_do());
    EXPECT_TRUE(scu::isActiveState(gp, "Open"));
    gp->jawLeftState.position.assign(1, gp->jawOpenPosition - 0.1);
    gp->jawRightState.position.assign(1, gp->jawOpenPosition - 0.1);
    gp->process_event(e_do());
    EXPECT_TRUE(scu::isActiveState(gp, "NotOpen"));
    gp->jawLeftState.position.assign(1, gp->jawOpenPosition + 0.1);
    gp->jawRightState.position.assign(1, gp->jawOpenPosition + 0.1);
    gp->process_event(e_do());
    EXPECT_TRUE(scu::isActiveState(gp, "Open"));
}

TEST_F(GripperPositionStateMachineTest, Force)
{
    //! event driven
    EXPECT_TRUE(scu::isActiveState(gp, "LessThanForceThreshold"));
    EXPECT_FLOAT_EQ(gp->strongForce, gp->desiredForce);
    gp->process_event(e_moreThanForceThreshold());
    EXPECT_TRUE(scu::isActiveState(gp, "MoreThanForceThreshold"));
    EXPECT_FLOAT_EQ(gp->weakForce, gp->desiredForce);
    gp->process_event(e_lessThanForceThreshold());
    EXPECT_TRUE(scu::isActiveState(gp, "LessThanForceThreshold"));

    //! data driven
    gp->jawLeftState.position.assign(1, gp->jawPositionForceThreshold + 0.1);
    gp->jawRightState.position.assign(1, gp->jawPositionForceThreshold + 0.1);
    gp->process_event(e_do());
    EXPECT_TRUE(scu::isActiveState(gp, "MoreThanForceThreshold"));
    gp->jawLeftState.position.assign(1, gp->jawPositionForceThreshold - 0.1);
    gp->jawRightState.position.assign(1, gp->jawPositionForceThreshold - 0.1);
    gp->process_event(e_do());
    EXPECT_TRUE(scu::isActiveState(gp, "LessThanForceThreshold"));
    gp->jawLeftState.position.assign(1, gp->jawPositionForceThreshold + 0.1);
    gp->jawRightState.position.assign(1, gp->jawPositionForceThreshold + 0.1);
    gp->process_event(e_do());
    EXPECT_TRUE(scu::isActiveState(gp, "MoreThanForceThreshold"));
}

TEST_F(GripperPositionStateMachineTest, SetParameters)
{
    GripperSetpoint setPoint;
    setPoint.jawPosition       = 10;
    setPoint.jawPositionDelta  = 10;
    setPoint.expectedLoad      = 10;
    setPoint.expectedLoadDelta = 10;

    EXPECT_TRUE(scu::isActiveState(gp, "Inactive"));
    gp->process_event(c_setParameters(setPoint));

    EXPECT_FLOAT_EQ(10, gp->setPoint.jawPosition);
    EXPECT_FLOAT_EQ(10, gp->setPoint.jawPositionDelta);
    EXPECT_FLOAT_EQ(10, gp->setPoint.expectedLoad);
    EXPECT_FLOAT_EQ(10, gp->setPoint.expectedLoad);

    setPoint.jawPosition       = 15;
    setPoint.jawPositionDelta  = 15;
    setPoint.expectedLoad      = 15;
    setPoint.expectedLoadDelta = 15;

    gp->process_event(e_ready());
    EXPECT_TRUE(scu::isActiveState(gp, "Listening"));
    gp->process_event(c_setParameters(setPoint));

    EXPECT_FLOAT_EQ(15, gp->setPoint.jawPosition);
    EXPECT_FLOAT_EQ(15, gp->setPoint.jawPositionDelta);
    EXPECT_FLOAT_EQ(15, gp->setPoint.expectedLoad);
    EXPECT_FLOAT_EQ(15, gp->setPoint.expectedLoad);

    //! Parameters can only be changed when Inactive or Listening
    setPoint.jawPosition       = 20;
    setPoint.jawPositionDelta  = 20;
    setPoint.expectedLoad      = 20;
    setPoint.expectedLoadDelta = 20;

    gp->process_event(c_set());
    EXPECT_FALSE(scu::isActiveState(gp, "Inactive"));
    EXPECT_FALSE(scu::isActiveState(gp, "Listening"));
    gp->process_event(c_setParameters(setPoint));

    EXPECT_FLOAT_EQ(15, gp->setPoint.jawPosition);
    EXPECT_FLOAT_EQ(15, gp->setPoint.jawPositionDelta);
    EXPECT_FLOAT_EQ(15, gp->setPoint.expectedLoad);
    EXPECT_FLOAT_EQ(15, gp->setPoint.expectedLoad);
}

TEST_F(GripperPositionStateMachineTest, Command)
{
    //! event driven
    EXPECT_TRUE(scu::isActiveState(gp, "Inactive"));
    gp->process_event(e_ready());
    EXPECT_TRUE(scu::isActiveState(gp, "Active"));
    EXPECT_TRUE(scu::isActiveState(gp, "Stopped"));
    EXPECT_TRUE(scu::isActiveState(gp, "Listening"));
    EXPECT_FLOAT_EQ(0, gp->jointCommand.desiredPositionVelocityLimit[0]);
    gp->process_event(e_unready());
    EXPECT_TRUE(scu::isActiveState(gp, "Inactive"));
    EXPECT_FLOAT_EQ(0, gp->jointCommand.desiredPositionVelocityLimit[0]);
    gp->process_event(e_ready());
    EXPECT_TRUE(scu::isActiveState(gp, "Active"));
    gp->process_event(e_faulted());
    EXPECT_TRUE(scu::isActiveState(gp, "Inactive"));
    //! can't be active if faulted
    gp->process_event(e_ready());
    EXPECT_FALSE(scu::isActiveState(gp, "Active"));

    gp->process_event(e_unready());
    gp->process_event(e_notFaulted());

    //! data driven
    gp->jointControlData.controlMode.state = nasa_r2_common_msgs::JointControlMode::DRIVE;
    gp->process_event(e_do());
    EXPECT_TRUE(scu::isActiveState(gp, "Active"));
    gp->jointControlData.controlMode.state = nasa_r2_common_msgs::JointControlMode::IGNORE;
    gp->process_event(e_do());
    EXPECT_TRUE(scu::isActiveState(gp, "Inactive"));
    //! @todo replace with data
    //! can't be active if faulted
    gp->process_event(e_faulted());
    gp->jointControlData.controlMode.state = nasa_r2_common_msgs::JointControlMode::DRIVE;
    gp->process_event(e_do());
    EXPECT_FALSE(scu::isActiveState(gp, "Active"));    
}

TEST_F(GripperPositionStateMachineTest, Movement)
{
    //! event driven
    gp->process_event(e_ready());
    EXPECT_TRUE(scu::isActiveState(gp, "Stopped"));
    EXPECT_FLOAT_EQ(0, gp->jointCommand.desiredPositionVelocityLimit[0]);

    gp->process_event(e_open());
    EXPECT_TRUE(scu::isActiveState(gp, "Opening"));
    //EXPECT_FLOAT_EQ(gp->encoderOpenVelocity, gp->jointCommand.desiredPositionVelocityLimit[0]);
    gp->process_event(e_stop());
    EXPECT_TRUE(scu::isActiveState(gp, "Stopped"));
    //EXPECT_FLOAT_EQ(0, gp->jointCommand.desiredPositionVelocityLimit[0]);
    gp->process_event(e_open());
    EXPECT_TRUE(scu::isActiveState(gp, "Opening"));
    gp->process_event(e_isOpen());
    EXPECT_TRUE(scu::isActiveState(gp, "Stopped"));

    gp->process_event(e_close());
    EXPECT_TRUE(scu::isActiveState(gp, "Closing"));
    //EXPECT_FLOAT_EQ(gp->encoderCloseVelocity, gp->jointCommand.desiredPositionVelocityLimit[0]);
    gp->process_event(e_stop());
    EXPECT_TRUE(scu::isActiveState(gp, "Stopped"));
    //EXPECT_FLOAT_EQ(0, gp->jointCommand.desiredPositionVelocityLimit[0]);
    gp->process_event(e_close());
    EXPECT_TRUE(scu::isActiveState(gp, "Closing"));
    gp->process_event(e_isClosed());
    EXPECT_TRUE(scu::isActiveState(gp, "Stopped"));

    gp->process_event(e_open());
    EXPECT_TRUE(scu::isActiveState(gp, "Opening"));
    gp->process_event(e_close());
    EXPECT_TRUE(scu::isActiveState(gp, "Closing"));
    gp->process_event(e_open());
    EXPECT_TRUE(scu::isActiveState(gp, "Opening"));

    //! data driven
    //! @todo check currents
    gp->process_event(e_stop());
    gp->process_event(e_isNotOpen());
    gp->jointControlData.controlMode.state = nasa_r2_common_msgs::JointControlMode::DRIVE;
    gp->process_event(e_open());
    EXPECT_TRUE(scu::isActiveState(gp, "Opening"));
    gp->encoderState.position.assign(1, 16);
    gp->jawLeftState.position.assign(1, 0.2);
    gp->jawRightState.position.assign(1, 0.2);
    gp->process_event(e_do());
    gp->process_event(e_do());
    gp->process_event(e_do());
    gp->process_event(e_do());
    //EXPECT_FLOAT_EQ(gp->encoderOpenPosition+10, gp->jointCommand.desiredPosition[0]);
    //gp->encoderState.position.assign(1, gp->encoderOpenPosition);
    gp->jawLeftState.position.assign(1, 0.55);
    gp->jawRightState.position.assign(1, 0.55);
    gp->process_event(e_do());
    gp->process_event(e_do());
    gp->process_event(e_do());
    gp->process_event(e_do());
    //EXPECT_FLOAT_EQ(gp->encoderOpenPosition, gp->jointCommand.desiredPosition[0]);
    EXPECT_TRUE(scu::isActiveState(gp, "Stopped"));
    gp->process_event(e_close());
    EXPECT_TRUE(scu::isActiveState(gp, "Closing"));
    gp->process_event(e_do());
    //EXPECT_FLOAT_EQ(gp->encoderClosedPosition-10, gp->jointCommand.desiredPosition[0]);
    gp->encoderState.position.assign(1, gp->encoderClosedPosition);
    gp->process_event(e_do());
    //EXPECT_FLOAT_EQ(gp->encoderClosedPosition, gp->jointCommand.desiredPosition[0]);
    //EXPECT_TRUE(scu::isActiveState(gp, "Stopped"));
}

TEST_F(GripperPositionStateMachineTest, ActionSet)
{
    //! event driven by e_still
    gp->process_event(e_ready());
    gp->process_event(c_set());
    EXPECT_TRUE(scu::isActiveState(gp, "OpenWide"));
    gp->process_event(e_moving());
    gp->process_event(i_isOpen_open());
    gp->process_event(e_moreThanDesiredJawPosition());
    gp->process_event(e_stalled());
    EXPECT_TRUE(scu::isActiveState(gp, "JawPositionCheck"));
    gp->process_event(e_atDesiredJawPosition());
    //! Done should never last, it auto-transitions to Listening.
    EXPECT_FALSE(scu::isActiveState(gp, "Done"));
    EXPECT_TRUE(scu::isActiveState(gp, "Listening"));

    //! event driven by e_isOpen
    gp->process_event(e_isNotOpen());
    gp->process_event(e_lessThanDesiredJawPosition());
    EXPECT_TRUE(scu::isActiveState(gp, "NotOpen"));
    EXPECT_TRUE(scu::isActiveState(gp, "LessThanDesiredJawPosition"));
    gp->process_event(c_set());
    EXPECT_TRUE(scu::isActiveState(gp, "OpenWide"));
    gp->process_event(e_moving());
    gp->process_event(e_moreThanDesiredJawPosition());
    gp->process_event(e_isOpen());
    EXPECT_TRUE(scu::isActiveState(gp, "JawPositionCheck"));
    gp->process_event(e_atDesiredJawPosition());
    //! Done should never last, it auto-transitions to Listening.
    EXPECT_FALSE(scu::isActiveState(gp, "Done"));
    EXPECT_TRUE(scu::isActiveState(gp, "Listening"));

    //! already Open
    EXPECT_TRUE(scu::isActiveState(gp, "Open"));
    gp->process_event(e_moreThanDesiredJawPosition());
    EXPECT_TRUE(scu::isActiveState(gp, "MoreThanDesiredJawPosition"));
    gp->process_event(c_set());
    EXPECT_FALSE(scu::isActiveState(gp, "OpenWide"));
    EXPECT_TRUE(scu::isActiveState(gp, "JawPositionCheck"));
    gp->process_event(e_atDesiredJawPosition());
    EXPECT_FALSE(scu::isActiveState(gp, "Done"));
    EXPECT_TRUE(scu::isActiveState(gp, "Listening"));

    //! already Open and AtDesiredJawPosition
    EXPECT_TRUE(scu::isActiveState(gp, "Open"));
    EXPECT_TRUE(scu::isActiveState(gp, "AtDesiredJawPosition"));
    gp->process_event(c_set());
    EXPECT_FALSE(scu::isActiveState(gp, "OpenWide"));
    EXPECT_FALSE(scu::isActiveState(gp, "JawPositionCheck"));
    EXPECT_FALSE(scu::isActiveState(gp, "Done"));
    EXPECT_TRUE(scu::isActiveState(gp, "Listening"));    

    //! data driven
    gp->setPoint.jawPosition = 0.25;
    gp->setPoint.jawPositionDelta = 0.100001;
    gp->jointControlData.controlMode.state = nasa_r2_common_msgs::JointControlMode::DRIVE;
    gp->process_event(e_isNotOpen());
    gp->process_event(e_lessThanDesiredJawPosition());
    EXPECT_TRUE(scu::isActiveState(gp, "NotOpen"));
    EXPECT_TRUE(scu::isActiveState(gp, "LessThanDesiredJawPosition"));
    gp->process_event(c_set());
    EXPECT_TRUE(scu::isActiveState(gp, "OpenWide"));
    gp->encoderState.velocity.assign(1, 10);
    gp->process_event(e_do());
    gp->jawLeftState.position.assign(1, gp->jawOpenPosition);
    gp->jawRightState.position.assign(1, gp->jawOpenPosition);
    gp->process_event(e_do());
    EXPECT_TRUE(scu::isActiveState(gp, "MoreThanDesiredJawPosition"));
    EXPECT_TRUE(scu::isActiveState(gp, "JawPositionCheck"));
    gp->encoderState.velocity.assign(1, 0);
    gp->jawLeftState.position.assign(1, 0.25);
    gp->jawRightState.position.assign(1, 0.25);
    gp->process_event(e_do());
    EXPECT_TRUE(scu::isActiveState(gp, "Listening"));
}

TEST_F(GripperPositionStateMachineTest, ActionLock)
{
    //! event driven
    gp->process_event(e_ready());
    gp->process_event(e_unlocked());
    gp->process_event(e_moreThanDesiredJawPosition());
    gp->process_event(c_lock());
    EXPECT_TRUE(scu::isActiveState(gp, "Close"));
    gp->process_event(e_atDesiredJawPosition());
    EXPECT_TRUE(scu::isActiveState(gp, "LockCheck"));
    gp->process_event(e_locked());
    EXPECT_TRUE(scu::isActiveState(gp, "LockCheck"));
    gp->process_event(e_stalled());
    //! Done should never last, it auto-transitions to Listening.
    EXPECT_FALSE(scu::isActiveState(gp, "Done"));
    EXPECT_TRUE(scu::isActiveState(gp, "Listening"));

    //! already at desired jaw position
    gp->process_event(e_unlocked());
    gp->process_event(c_lock());
    EXPECT_FALSE(scu::isActiveState(gp, "Close"));
    EXPECT_TRUE(scu::isActiveState(gp, "LockCheck"));
    gp->process_event(e_locked());
    gp->process_event(e_stalled());
    EXPECT_FALSE(scu::isActiveState(gp, "Done"));
    EXPECT_TRUE(scu::isActiveState(gp, "Listening"));

    //! already at desired jaw position and locked
    gp->process_event(c_lock());
    gp->process_event(e_stalled());
    EXPECT_FALSE(scu::isActiveState(gp, "Close"));
    EXPECT_FALSE(scu::isActiveState(gp, "LockCheck"));
    EXPECT_FALSE(scu::isActiveState(gp, "Done"));
    EXPECT_TRUE(scu::isActiveState(gp, "Listening"));    

    //! data driven
    gp->jointControlData.controlMode.state = nasa_r2_common_msgs::JointControlMode::DRIVE;
    gp->process_event(e_unlocked());
    gp->process_event(e_moreThanDesiredJawPosition());
    gp->process_event(c_lock());
    EXPECT_TRUE(scu::isActiveState(gp, "Close"));
    //! @todo replace with data
    gp->process_event(e_atDesiredJawPosition());
    EXPECT_TRUE(scu::isActiveState(gp, "LockCheck"));
    gp->process_event(e_locked());
    gp->process_event(e_stalled());
    EXPECT_TRUE(scu::isActiveState(gp, "Listening"));    
}

TEST_F(GripperPositionStateMachineTest, ActionRelease)
{
    //! event driven
    //! stall transition
    gp->process_event(e_ready());
    gp->process_event(c_release());
    EXPECT_TRUE(scu::isActiveState(gp, "OpenUntilStall"));
    gp->process_event(e_stalled());
    //! Done should never last, it auto-transitions to Listening.
    EXPECT_FALSE(scu::isActiveState(gp, "Done"));
    EXPECT_TRUE(scu::isActiveState(gp, "Listening"));

    //! isOpen transition
    gp->process_event(c_release());
    EXPECT_TRUE(scu::isActiveState(gp, "OpenUntilStall"));
    gp->process_event(e_isOpen());
    EXPECT_FALSE(scu::isActiveState(gp, "Done"));
    EXPECT_TRUE(scu::isActiveState(gp, "Listening"));

    //! already Open
    gp->process_event(e_isOpen());
    gp->process_event(c_release());
    EXPECT_FALSE(scu::isActiveState(gp, "OpenUntilStall"));
    EXPECT_FALSE(scu::isActiveState(gp, "Done"));
    EXPECT_TRUE(scu::isActiveState(gp, "Listening"));

    //! data driven
    gp->jointControlData.controlMode.state = nasa_r2_common_msgs::JointControlMode::DRIVE;
    gp->process_event(e_isNotOpen());
    gp->process_event(e_notStalled());
    gp->process_event(c_release());
    EXPECT_TRUE(scu::isActiveState(gp, "OpenUntilStall"));
    //! @todo set encoder and jaw angles to generate proper currentLimit
    setFloat(gp->measuredCurrentName, 10);
    gp->encoderState.velocity.assign(1, 0);
    gp->process_event(e_do());
    EXPECT_TRUE(scu::isActiveState(gp, "Listening"));
}

TEST_F(GripperPositionStateMachineTest, ActionInterrupt)
{
    gp->process_event(e_ready());
    gp->process_event(c_set());
    EXPECT_TRUE(scu::isActiveState(gp, "OpenWide"));
    gp->process_event(e_moreThanDesiredJawPosition());
    gp->process_event(c_lock());
    EXPECT_TRUE(scu::isActiveState(gp, "Close"));
    gp->process_event(c_release());
    EXPECT_TRUE(scu::isActiveState(gp, "OpenUntilStall"));
}

TEST_F(GripperPositionStateMachineTest, ActionOpenWideFailure)
{
    //! event driven
    gp->process_event(e_ready());
    gp->process_event(c_set());
    EXPECT_TRUE(scu::isActiveState(gp, "OpenWide"));
    gp->process_event(e_stalled());
    EXPECT_TRUE(scu::isActiveState(gp, "Listening"));

    gp->process_event(c_set());
    EXPECT_TRUE(scu::isActiveState(gp, "OpenWide"));
    gp->process_event(e_moreThanDesiredJawPosition());
    gp->process_event(i_isOpen_open());
    EXPECT_FALSE(scu::isActiveState(gp, "NotOpen"));
    gp->process_event(e_stalled());
    EXPECT_FALSE(scu::isActiveState(gp, "Listening"));
}

TEST_F(GripperPositionStateMachineTest, ActionJawPositionCheckFailure)
{
    //! event driven
    gp->process_event(e_ready());
    gp->process_event(c_set());
    EXPECT_TRUE(scu::isActiveState(gp, "OpenWide"));
    gp->process_event(e_moreThanDesiredJawPosition());
    gp->process_event(e_isOpen());
    EXPECT_TRUE(scu::isActiveState(gp, "JawPositionCheck"));
    gp->process_event(e_stalled());
    EXPECT_TRUE(scu::isActiveState(gp, "Listening"));
    
    gp->process_event(e_isNotOpen());
    gp->process_event(c_set());
    EXPECT_TRUE(scu::isActiveState(gp, "OpenWide"));
    gp->process_event(e_isOpen());
    EXPECT_TRUE(scu::isActiveState(gp, "JawPositionCheck"));
    gp->process_event(i_atDesiredJawPosition_position());
    EXPECT_FALSE(scu::isActiveState(gp, "MoreThanDesiredJawPosition"));
    gp->process_event(e_stalled());
    EXPECT_FALSE(scu::isActiveState(gp, "Listening"));

    //! @todo data driven
}

TEST_F(GripperPositionStateMachineTest, ActionCloseFailure)
{
    //! event driven
    gp->process_event(e_ready());
    gp->process_event(e_moreThanDesiredJawPosition());
    gp->process_event(c_lock());
    EXPECT_TRUE(scu::isActiveState(gp, "Close"));
    gp->process_event(e_stalled());
    EXPECT_TRUE(scu::isActiveState(gp, "Listening"));
    
    gp->process_event(c_lock());
    EXPECT_TRUE(scu::isActiveState(gp, "Close"));
    gp->process_event(i_atDesiredJawPosition_position());
    EXPECT_FALSE(scu::isActiveState(gp, "MoreThanDesiredJawPosition"));
    gp->process_event(e_stalled());
    EXPECT_FALSE(scu::isActiveState(gp, "Listening"));

    //! @todo data driven
}

TEST_F(GripperPositionStateMachineTest, ActionLockCheckFailure)
{
    //! event driven
    gp->process_event(e_ready());
    gp->process_event(e_unlocked());
    gp->process_event(e_moreThanDesiredJawPosition());
    gp->process_event(c_lock());
    EXPECT_TRUE(scu::isActiveState(gp, "Close"));
    gp->process_event(e_atDesiredJawPosition());
    EXPECT_TRUE(scu::isActiveState(gp, "LockCheck"));
    gp->process_event(e_stalled());
    EXPECT_TRUE(scu::isActiveState(gp, "Listening"));
    
    gp->process_event(e_moreThanDesiredJawPosition());
    gp->process_event(c_lock());
    EXPECT_TRUE(scu::isActiveState(gp, "Close"));
    gp->process_event(e_atDesiredJawPosition());
    EXPECT_TRUE(scu::isActiveState(gp, "LockCheck"));
    gp->process_event(i_locked_lock());
    EXPECT_FALSE(scu::isActiveState(gp, "Unlocked"));
    gp->process_event(e_stalled());
    EXPECT_TRUE(scu::isActiveState(gp, "Listening"));

    //! @todo data driven
}

int main(int argc, char **argv){
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}