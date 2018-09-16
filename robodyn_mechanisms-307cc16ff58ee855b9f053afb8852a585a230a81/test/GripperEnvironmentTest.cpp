#include <gtest/gtest.h>
#include "nasa_robodyn_mechanisms_core/GripperEnvironment.h"
#include <ros/package.h>

class GripperEnvironmentTest : public ::testing::Test 
{
protected:
    virtual void SetUp()
    {
        ros::Time::init();
     
        grippers.push_back("/r2/left_leg/gripper/joint0");
        grippers.push_back("/r2/right_leg/gripper/joint0");
        
        ge.initializeMap(grippers);
        handrailComp = "HandrailRendezvous";
        seattrackComp = "SeattrackRendezvous";
        ge.setComponentNames(handrailComp, seattrackComp);
        
        gripperState.name.push_back("/r2/left_leg/gripper/joint0");
		gripperState.name.push_back("/r2/right_leg/gripper/joint0");
		gripperState.locked.push_back(1);
		gripperState.locked.push_back(0);
		gripperState.loaded.push_back(1);
		gripperState.loaded.push_back(1);
		
		baseFrames.data.push_back("/r2/left_leg/gripper/tip");
		
		goalStati.header.frame_id = "GripperPositionManager";
		goalStati.status_list.push_back(goalStat);
		goalStat.goal_id.id = "/r2/right_leg/gripper/joint0";
        goalStat.status = actionlib_msgs::GoalStatus::PENDING;  
        goalStati.status_list.push_back(goalStat);
        goalStat.goal_id.id = "LeftHandrailRendezvous";
        goalStat.status = actionlib_msgs::GoalStatus::ABORTED;  
        goalStati.status_list.push_back(goalStat);
        goalStat.goal_id.id = "RightSeattrackRendezvous";
        goalStat.status = actionlib_msgs::GoalStatus::SUCCEEDED;  
        goalStati.status_list.push_back(goalStat);
        
        tempPose.position.x = tempPose.position.y = tempPose.position.z = 0.;
        tempPose.orientation.x = tempPose.orientation.y = tempPose.orientation.z = 0.0;
        tempPose.orientation.w = 1.;
        poseMsg.name.push_back("/r2/left_leg/gripper/tip");
        poseMsg.positions.push_back(tempPose);
        tempPose.position.x = 1.;
        poseMsg.name.push_back("/r2/right_leg/gripper/tip");
        poseMsg.positions.push_back(tempPose);
             
    }

    virtual void TearDown()
    {
    }

	GripperEnvironment ge;
	std::vector<std::string> grippers;
	std::string handrailComp, seattrackComp;
    nasa_r2_common_msgs::GripperPositionState gripperState;
    nasa_r2_common_msgs::StringArray baseFrames;
    nasa_r2_common_msgs::GripperEnvironment gripperEnv;
    actionlib_msgs::GoalStatusArray goalStati;
    actionlib_msgs::GoalStatus goalStat;
    GripperEnvironment::GripperEnvInfo info;
    nasa_r2_common_msgs::PoseState poseMsg;
    geometry_msgs::Pose tempPose;
};

TEST_F(GripperEnvironmentTest, setUserCommandTest)
{
	ge.setUserCommand(grippers[0], nasa_r2_common_msgs::GripperEnvironment::HANDRAIL);
	EXPECT_TRUE(ge.getMapEntry(grippers[0], info));
	EXPECT_EQ(nasa_r2_common_msgs::GripperEnvironment::HANDRAIL, info.userCommand);
    
    ge.setUserCommand("blah", 1);
    EXPECT_FALSE(ge.getMapEntry("blah", info));
    EXPECT_EQ(nasa_r2_common_msgs::GripperEnvironment::HANDRAIL, info.userCommand);

}

TEST_F(GripperEnvironmentTest, populateBaseFrameInfoTest)
{
    
    ge.populateBaseFrameInfo(baseFrames);

    EXPECT_EQ("/r2/left_leg/gripper/tip", ge.getBaseFrame());

}

TEST_F(GripperEnvironmentTest, populateGripperStateInfoTest)
{
	 ge.populateGripperStateInfo(gripperState);
	 
	 EXPECT_TRUE(ge.getMapEntry(gripperState.name[0], info));
	 EXPECT_EQ(1, info.gripperState);
	 EXPECT_TRUE(ge.getMapEntry(gripperState.name[1], info));
	 EXPECT_EQ(0, info.gripperState);
}

TEST_F(GripperEnvironmentTest, populateGoalStatusTest)
{
	 ge.populateGoalStatus(goalStati);
	 EXPECT_EQ(0, ge.getGoalStatusArraySize());
	 
	 goalStati.header.frame_id = handrailComp;
	 ge.populateGoalStatus(goalStati);
	 EXPECT_EQ(1, ge.getGoalStatusArraySize());
	 
	 goalStati.header.frame_id = "DustinSmellsFunny";
	 ge.populateGoalStatus(goalStati);
	 EXPECT_EQ(0, ge.getGoalStatusArraySize());
	 
	 goalStati.header.frame_id = seattrackComp;
	 ge.populateGoalStatus(goalStati);
	 EXPECT_EQ(1, ge.getGoalStatusArraySize());
}

TEST_F(GripperEnvironmentTest, populatePoseStateInfoTest)
{
	 ge.populatePoseStateInfo(poseMsg);
	 
	 EXPECT_TRUE( ge.getMapEntry(ge.BaseFrameToGripper(poseMsg.name[1]), info));
	 EXPECT_EQ(KDL::Frame(), info.frame);
	 
	 ge.populateBaseFrameInfo(baseFrames);
	 ge.populatePoseStateInfo(poseMsg);
	 
	 EXPECT_TRUE( ge.getMapEntry(ge.BaseFrameToGripper(poseMsg.name[1]), info));
	 EXPECT_NE(KDL::Frame(), info.frame);
	 
}

TEST_F(GripperEnvironmentTest, createGripperEnvironmentMsgTest)
{
	 ge.createGripperEnvironmentMsg(gripperEnv);
	 
	 EXPECT_EQ(2, gripperEnv.name.size());
	 ASSERT_EQ(2, gripperEnv.environment.size());
	 EXPECT_EQ(nasa_r2_common_msgs::GripperEnvironment::NONE, gripperEnv.environment[0]);
	 EXPECT_EQ(nasa_r2_common_msgs::GripperEnvironment::NONE, gripperEnv.environment[1]);
}

TEST_F(GripperEnvironmentTest, updateTest)
{
	ge.populateBaseFrameInfo(baseFrames);
	ge.populateGripperStateInfo(gripperState);
	ge.update();
	ASSERT_TRUE(ge.getMapEntry("/r2/left_leg/gripper/joint0", info));
	EXPECT_EQ(nasa_r2_common_msgs::GripperEnvironment::HANDRAIL, info.environment);
	EXPECT_EQ(1, info.highForceSubState);
	ASSERT_TRUE(ge.getMapEntry("/r2/right_leg/gripper/joint0", info));
	EXPECT_EQ(nasa_r2_common_msgs::GripperEnvironment::FREESPACE, info.environment);
	
	gripperState.locked[0] = 0;
	baseFrames.data.clear();
	ge.populateBaseFrameInfo(baseFrames);
	ge.populateGripperStateInfo(gripperState);
	goalStati.header.frame_id = seattrackComp;
	ge.populateGoalStatus(goalStati);
	ge.populatePoseStateInfo(poseMsg);
	ge.update();
	ASSERT_TRUE(ge.getMapEntry("/r2/left_leg/gripper/joint0", info));
	EXPECT_EQ(nasa_r2_common_msgs::GripperEnvironment::FREESPACE, info.environment);
	EXPECT_EQ(0, info.highForceSubState);
	ASSERT_TRUE(ge.getMapEntry("/r2/right_leg/gripper/joint0", info));
	EXPECT_EQ(nasa_r2_common_msgs::GripperEnvironment::SEATTRACK, info.environment);
	EXPECT_EQ(0, info.highForceSubState);
	
	baseFrames.data.push_back("/r2/left_leg/gripper/tip");
	poseMsg.positions[1].position.y = 2.0;
	ge.populateBaseFrameInfo(baseFrames);
	ge.populatePoseStateInfo(poseMsg);
	ge.update();
	ASSERT_TRUE(ge.getMapEntry("/r2/right_leg/gripper/joint0", info));
	EXPECT_EQ(nasa_r2_common_msgs::GripperEnvironment::FREESPACE, info.environment);
	
	ge.setUserCommand("/r2/left_leg/gripper/joint0", nasa_r2_common_msgs::GripperEnvironment::HANDRAIL);
	ge.update();
	ASSERT_TRUE(ge.getMapEntry("/r2/left_leg/gripper/joint0", info));
	EXPECT_EQ(nasa_r2_common_msgs::GripperEnvironment::HANDRAIL, info.environment);
	EXPECT_EQ(0, info.highForceSubState);
	
	gripperState.locked[0] = 1;
	ge.populateGripperStateInfo(gripperState);
	ge.populatePoseStateInfo(poseMsg);
	ge.update();
	ASSERT_TRUE(ge.getMapEntry("/r2/left_leg/gripper/joint0", info));
	EXPECT_EQ(nasa_r2_common_msgs::GripperEnvironment::HANDRAIL, info.environment);
	EXPECT_EQ(1, info.highForceSubState);
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
