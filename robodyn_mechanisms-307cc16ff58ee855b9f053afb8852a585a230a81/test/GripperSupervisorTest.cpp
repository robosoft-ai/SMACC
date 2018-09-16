#include <gtest/gtest.h>
#include "nasa_robodyn_mechanisms_core/GripperSupervisor.h"
#include <ros/package.h>

class GripperSupervisorTest : public ::testing::Test 
{
protected:
    virtual void SetUp()
    {
        ros::Time::init();
     
        grippers.push_back("/r2/left_leg/gripper/joint0");
        grippers.push_back("/r2/right_leg/gripper/joint0");
        
        gripperState.name.push_back("/r2/left_leg/gripper/joint0");
		gripperState.name.push_back("/r2/right_leg/gripper/joint0");
		gripperState.locked.push_back(1);
		gripperState.locked.push_back(0);
		gripperState.loaded.push_back(1);
		gripperState.loaded.push_back(1);
		
		baseFrames.data.push_back("/r2/left_leg/gripper/tip");
		
		gripperEnv.name.push_back("/r2/left_leg/gripper/joint0");
		gripperEnv.name.push_back("/r2/right_leg/gripper/joint0");
		gripperEnv.environment.push_back(nasa_r2_common_msgs::GripperEnvironment::HANDRAIL);
		gripperEnv.environment.push_back(nasa_r2_common_msgs::GripperEnvironment::FREESPACE);
		
		goalStat.goal_id.id = "/r2/left_leg/gripper/joint0";
		goalStat.goal_id.stamp = ros::Time::now();
		goalStat.status = actionlib_msgs::GoalStatus::SUCCEEDED;
		goalStati.status_list.push_back(goalStat);
		goalStat.goal_id.id = "/r2/right_leg/gripper/joint0";
        goalStat.status = actionlib_msgs::GoalStatus::PENDING;  
        goalStati.status_list.push_back(goalStat);
        goalStat.status = actionlib_msgs::GoalStatus::ABORTED;  
        goalStati.status_list.push_back(goalStat);
        
        jntMode.controlMode.state = nasa_r2_common_msgs::JointControlMode::PARK;
        jntModeList.joint.push_back("/r2/left_leg/gripper/joint0");
        jntModeList.data.push_back(jntMode);
        jntMode.controlMode.state = nasa_r2_common_msgs::JointControlMode::FAULTED;
        jntModeList.joint.push_back("/r2/right_leg/gripper/joint0");
        jntModeList.data.push_back(jntMode);
        jntMode.controlMode.state = nasa_r2_common_msgs::JointControlMode::DRIVE;
        jntModeList.joint.push_back("joint1");
        jntModeList.data.push_back(jntMode);
        
        gripCmd.name.push_back("/r2/left_leg/gripper/joint0");
        gripCmd.setpoint.push_back("null");
        gripCmd.command.push_back("release");
        gripCmd.name.push_back("/r2/right_leg/gripper/joint0");
        gripCmd.setpoint.push_back("null");
        gripCmd.command.push_back("lock");
             
    }

    virtual void TearDown()
    {
    }

	GripperSupervisor gs;
	std::vector<std::string> grippers;
    nasa_r2_common_msgs::GripperPositionState gripperState;
    nasa_r2_common_msgs::StringArray baseFrames;
    nasa_r2_common_msgs::GripperEnvironment gripperEnv;
    actionlib_msgs::GoalStatusArray goalStati;
    actionlib_msgs::GoalStatus goalStat;
    nasa_r2_common_msgs::JointControlDataArray jntModeList;
    nasa_r2_common_msgs::JointControlData jntMode;
    nasa_r2_common_msgs::GripperPositionCommand gripCmd;
};

TEST_F(GripperSupervisorTest, InitializeMapsTest)
{

    // Initialize maps with test tree
    gs.InitializeMaps(grippers);
 
    ASSERT_FALSE( gs.endEffectorMap.empty());
    EXPECT_EQ ( 0, gs.endEffectorMap.count("joint1"));
    EXPECT_EQ ( 1, gs.gripperEnvMap.count("/r2/left_leg/gripper/joint0"));
    EXPECT_EQ ( nasa_r2_common_msgs::GripperEnvironment::NONE, gs.gripperEnvMap["/r2/right_leg/gripper/joint0"]);
    EXPECT_EQ ( IDLE, gs.getState("/r2/left_leg/gripper/joint0"));
    EXPECT_EQ ( 0, gs.gripperJointModeMap.count("link1"));
}

TEST_F(GripperSupervisorTest, PopulateGripperStateInfoTest)
{
    gs.InitializeMaps(grippers);
	
    gs.PopulateGripperStateInfo(gripperState);
    
    EXPECT_EQ( 1, gs.endEffectorMap["/r2/left_leg/gripper/joint0"].locked );
    EXPECT_EQ( 1, gs.endEffectorMap["/r2/right_leg/gripper/joint0"].loaded );

}

TEST_F(GripperSupervisorTest, PopulateBaseFrameInfoTest)
{
    gs.InitializeMaps(grippers);
    
    gs.PopulateBaseFrameInfo(baseFrames);

    EXPECT_EQ(1, gs.baseFrameList.size());
    EXPECT_EQ("/r2/left_leg/gripper/tip", gs.baseFrameList[0]);
    
    gs.PopulateBaseFrameInfo(baseFrames);
    EXPECT_EQ(1, gs.baseFrameList.size());

}

TEST_F(GripperSupervisorTest, PopulateEnvironmentInfoTest)
{
	 gs.InitializeMaps(grippers);
	 
	 gs.PopulateEnvironmentInfo(gripperEnv);
	 
	 EXPECT_EQ(0, gs.gripperEnvMap.count("joint1"));
	 EXPECT_EQ(1, gs.gripperEnvMap.count("/r2/left_leg/gripper/joint0"));
	 EXPECT_EQ(nasa_r2_common_msgs::GripperEnvironment::FREESPACE, gs.gripperEnvMap["/r2/right_leg/gripper/joint0"]);
}

TEST_F(GripperSupervisorTest, PopulateGoalStatusTest)
{
	 gs.InitializeMaps(grippers);
	 
	 gs.PopulateGoalStatus(goalStati);
	 
	 EXPECT_EQ(2, gs.goalStatuses.status_list.size());
	 EXPECT_EQ(actionlib_msgs::GoalStatus::SUCCEEDED, gs.goalStatuses.status_list[0].status);
	 EXPECT_EQ("/r2/right_leg/gripper/joint0", gs.goalStatuses.status_list[1].goal_id.id);
}

TEST_F(GripperSupervisorTest, PopulateJointControlModeTest)
{
	 gs.InitializeMaps(grippers);
	 
	 gs.PopulateJointControlMode(jntModeList);
	 
	 EXPECT_EQ(0, gs.gripperJointModeMap.count("joint1"));
	 EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::PARK, gs.gripperJointModeMap["/r2/left_leg/gripper/joint0"]);
	 EXPECT_EQ(1, gs.gripperJointModeMap.count("/r2/right_leg/gripper/joint0"));
}

TEST_F(GripperSupervisorTest, PopulateGripperCommandsInTest)
{
	 gs.InitializeMaps(grippers);
	 
	 gs.PopulateGripperCommandsIn(gripCmd);
	 
	 EXPECT_EQ(2, gs.gripperCmds.name.size());
	 EXPECT_EQ("release", gs.gripperCmds.command[0]);
	 EXPECT_EQ("/r2/right_leg/gripper/joint0", gs.gripperCmds.name[1]);
}

TEST_F(GripperSupervisorTest, CheckTransitionsTest)
{
	
	nasa_r2_common_msgs::JointControlDataArray modeCmdOut;
	nasa_r2_common_msgs::GripperPositionCommand gripCmdOut;
	actionlib_msgs::GoalStatusArray statusOut;
	
	gs.InitializeMaps(grippers);
	gs.PopulateBaseFrameInfo(baseFrames);
	gs.PopulateEnvironmentInfo(gripperEnv);
	gs.setLimits(30, 255);
	 
	// check joint fault bit response
	gs.PopulateJointControlMode(jntModeList);
	gs.CheckTransitions(false);
	gs.PopulateJointControlCommandMsg(modeCmdOut);
	gs.PopulateGoalStatusMsg(statusOut);
	gs.PopulateGripperCommandMsg(gripCmdOut);
	EXPECT_EQ(FAULT, gs.getState("/r2/right_leg/gripper/joint0"));
	EXPECT_EQ(IDLE, gs.getState("/r2/left_leg/gripper/joint0"));
	ASSERT_EQ(1, modeCmdOut.joint.size());
	EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::PARK, modeCmdOut.data[0].controlMode.state);
	ASSERT_EQ(1, gripCmdOut.name.size());
	EXPECT_EQ("c_cancel", gripCmdOut.command[0]);
	EXPECT_EQ("null", gripCmdOut.setpoint[0]);
	
	// check running this with no commands (should be no change)
	gs.CheckTransitions(false);
	gs.PopulateJointControlCommandMsg(modeCmdOut);
	gs.PopulateGoalStatusMsg(statusOut);
	gs.PopulateGripperCommandMsg(gripCmdOut);
	EXPECT_EQ(FAULT, gs.getState("/r2/right_leg/gripper/joint0"));
	EXPECT_EQ(IDLE, gs.getState("/r2/left_leg/gripper/joint0"));
	ASSERT_EQ(0, modeCmdOut.joint.size());
	ASSERT_EQ(0, gripCmdOut.name.size());
	
	// Remove the fault from the current joint
	jntModeList.data[1].controlMode.state = nasa_r2_common_msgs::JointControlMode::PARK;
	gs.PopulateJointControlMode(jntModeList);
	gs.CheckTransitions(false);
	gs.PopulateJointControlCommandMsg(modeCmdOut);
	gs.PopulateGoalStatusMsg(statusOut);
	gs.PopulateGripperCommandMsg(gripCmdOut);
	EXPECT_EQ(IDLE, gs.getState("/r2/right_leg/gripper/joint0"));
	ASSERT_EQ(1, modeCmdOut.joint.size());
	EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::PARK, modeCmdOut.data[0].controlMode.state);
	ASSERT_EQ(0, gripCmdOut.name.size());
	 
	// Send in some goal status
	gs.PopulateGoalStatus(goalStati);
	gs.CheckTransitions(false);
	gs.PopulateJointControlCommandMsg(modeCmdOut);
	gs.PopulateGoalStatusMsg(statusOut);
	gs.PopulateGripperCommandMsg(gripCmdOut);
	EXPECT_EQ(IDLE, gs.getState("/r2/left_leg/gripper/joint0"));
	EXPECT_EQ(FAILURE, gs.getState("/r2/right_leg/gripper/joint0"));
	ASSERT_EQ(2, modeCmdOut.joint.size());
	EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::PARK, modeCmdOut.data[1].controlMode.state);
	ASSERT_EQ(0, gripCmdOut.name.size());
	
	// Send two gripper commands, one invalid
	gs.PopulateGripperCommandsIn(gripCmd);
	gs.CheckTransitions(false);
	gs.PopulateJointControlCommandMsg(modeCmdOut);
	gs.PopulateGoalStatusMsg(statusOut);
	gs.PopulateGripperCommandMsg(gripCmdOut);
	EXPECT_EQ(IDLE, gs.getState("/r2/left_leg/gripper/joint0"));
	EXPECT_EQ(FAILURE, gs.getState("/r2/right_leg/gripper/joint0"));
	ASSERT_EQ(0, modeCmdOut.joint.size());
	ASSERT_EQ(0, gripCmdOut.name.size());
	//EXPECT_EQ(actionlib_msgs::GoalStatus::REJECTED, statusOut.status_list[0].status);
	 
	// Reset the failed joint
	gripCmd.name.clear();
	gripCmd.setpoint.clear();
	gripCmd.command.clear();
    gripCmd.name.push_back("/r2/right_leg/gripper/joint0");
    gripCmd.setpoint.push_back("null");
    gripCmd.command.push_back("reset");
	gs.PopulateGripperCommandsIn(gripCmd);
	gs.CheckTransitions(false);
	gs.PopulateJointControlCommandMsg(modeCmdOut);
	gs.PopulateGoalStatusMsg(statusOut);
	gs.PopulateGripperCommandMsg(gripCmdOut);
	EXPECT_EQ(IDLE, gs.getState("/r2/right_leg/gripper/joint0"));
	ASSERT_EQ(1, modeCmdOut.joint.size());
	EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::PARK, modeCmdOut.data[0].controlMode.state);
	ASSERT_EQ(0, gripCmdOut.name.size());
	
	// close right_leg/gripper/joint0 
	gripCmd.name.clear();
	gripCmd.setpoint.clear();
	gripCmd.command.clear();
	gripCmd.name.push_back("/r2/right_leg/gripper/joint0");
    gripCmd.setpoint.push_back("closed");
    gripCmd.command.push_back("set");
	gs.PopulateGripperCommandsIn(gripCmd);
	gs.CheckTransitions(false);
	gs.PopulateJointControlCommandMsg(modeCmdOut);
	gs.PopulateGoalStatusMsg(statusOut);
	gs.PopulateGripperCommandMsg(gripCmdOut);
	EXPECT_EQ(SET, gs.getState("/r2/right_leg/gripper/joint0"));
	ASSERT_EQ(1, modeCmdOut.joint.size());
	EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::DRIVE, modeCmdOut.data[0].controlMode.state);
	EXPECT_EQ(0, gripCmdOut.name.size());
	
	// send drive command
	jntModeList.joint.clear();
	jntModeList.data.clear();
	jntMode.controlMode.state = nasa_r2_common_msgs::JointControlMode::DRIVE;
	jntModeList.joint.push_back("/r2/right_leg/gripper/joint0");
    jntModeList.data.push_back(jntMode);
    gs.PopulateJointControlMode(jntModeList);
	gs.CheckTransitions(false);
	gs.PopulateJointControlCommandMsg(modeCmdOut);
	gs.PopulateGoalStatusMsg(statusOut);
	gs.PopulateGripperCommandMsg(gripCmdOut);
	EXPECT_EQ(SET_PENDING, gs.getState("/r2/right_leg/gripper/joint0"));
	EXPECT_EQ(0, modeCmdOut.joint.size());
	ASSERT_EQ(1, gripCmdOut.name.size());
	EXPECT_EQ("c_set", gripCmdOut.command[0]);
	EXPECT_EQ(30, gripCmdOut.dutyCycle[0]);
	
	// send success
	goalStati.status_list.clear();
	goalStat.goal_id.id = "/r2/right_leg/gripper/joint0";
	goalStat.goal_id.stamp = ros::Time::now();
	goalStat.status = actionlib_msgs::GoalStatus::SUCCEEDED;
	goalStati.status_list.push_back(goalStat);
	gs.PopulateGoalStatus(goalStati);
	gs.CheckTransitions(false);
	gs.PopulateJointControlCommandMsg(modeCmdOut);
	gs.PopulateGoalStatusMsg(statusOut);
	gs.PopulateGripperCommandMsg(gripCmdOut);
	EXPECT_EQ(IDLE, gs.getState("/r2/right_leg/gripper/joint0"));
	EXPECT_EQ(0, modeCmdOut.joint.size());
	EXPECT_EQ(0, gripCmdOut.name.size());
	
	// lock right_leg/gripper/joint0
	gripCmd.name.clear();
	gripCmd.setpoint.clear();
	gripCmd.command.clear();
	gripCmd.name.push_back("/r2/right_leg/gripper/joint0");
    gripCmd.setpoint.push_back("Dustin smells funny");
    gripCmd.command.push_back("lock");
	gs.PopulateGripperCommandsIn(gripCmd);
	gs.CheckTransitions(false);
	gs.PopulateJointControlCommandMsg(modeCmdOut);
	gs.PopulateGoalStatusMsg(statusOut);
	gs.PopulateGripperCommandMsg(gripCmdOut);
	EXPECT_EQ(PENDING, gs.getState("/r2/right_leg/gripper/joint0"));
	ASSERT_EQ(1, gripCmdOut.name.size());
	EXPECT_EQ(0, modeCmdOut.joint.size());
	EXPECT_EQ("c_lock", gripCmdOut.command[0]);
	EXPECT_EQ("Dustin smells funny", gripCmdOut.setpoint[0]);
	EXPECT_EQ(255, gripCmdOut.dutyCycle[0]);
	
	// send success
	goalStati.status_list.clear();
	goalStat.goal_id.id = "/r2/right_leg/gripper/joint0";
	goalStat.goal_id.stamp = ros::Time::now();
	goalStat.status = actionlib_msgs::GoalStatus::SUCCEEDED;
	goalStati.status_list.push_back(goalStat);
	gs.PopulateGoalStatus(goalStati);
	gs.CheckTransitions(false);
	gs.PopulateJointControlCommandMsg(modeCmdOut);
	gs.PopulateGoalStatusMsg(statusOut);
	gs.PopulateGripperCommandMsg(gripCmdOut);
	EXPECT_EQ(IDLE, gs.getState("/r2/right_leg/gripper/joint0"));
	ASSERT_EQ(1, modeCmdOut.joint.size());
	EXPECT_EQ(0, gripCmdOut.name.size());
	EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::PARK, modeCmdOut.data[0].controlMode.state);
	
	// release left_leg/gripper/joint0
	baseFrames.data.push_back("/r2/right_leg/gripper/tip");
	gs.PopulateBaseFrameInfo(baseFrames);
	gripCmd.name.clear();
	gripCmd.setpoint.clear();
	gripCmd.command.clear();
	gripCmd.name.push_back("/r2/left_leg/gripper/joint0");
    gripCmd.setpoint.push_back("null");
    gripCmd.command.push_back("release");
    gs.PopulateGripperCommandsIn(gripCmd);
	gs.CheckTransitions(false);
	gs.PopulateJointControlCommandMsg(modeCmdOut);
	gs.PopulateGoalStatusMsg(statusOut);
	gs.PopulateGripperCommandMsg(gripCmdOut);
	EXPECT_EQ(RELEASE, gs.getState("/r2/left_leg/gripper/joint0"));
	ASSERT_EQ(1, modeCmdOut.joint.size());
	EXPECT_EQ(0, gripCmdOut.name.size());
	EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::DRIVE, modeCmdOut.data[0].controlMode.state);
	
	// send failure
	goalStati.status_list.clear();
	goalStat.goal_id.id = "/r2/left_leg/gripper/joint0";
	goalStat.goal_id.stamp = ros::Time::now();
	goalStat.status = actionlib_msgs::GoalStatus::ABORTED;
	goalStati.status_list.push_back(goalStat);
	gs.PopulateGoalStatus(goalStati);
	gs.CheckTransitions(false);
	gs.PopulateJointControlCommandMsg(modeCmdOut);
	gs.PopulateGoalStatusMsg(statusOut);
	gs.PopulateGripperCommandMsg(gripCmdOut);
	EXPECT_EQ(FAILURE, gs.getState("/r2/left_leg/gripper/joint0"));
	ASSERT_EQ(1, modeCmdOut.joint.size());
	EXPECT_EQ(0, gripCmdOut.name.size());
	EXPECT_EQ(nasa_r2_common_msgs::JointControlMode::PARK, modeCmdOut.data[0].controlMode.state);
	
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
