#ifndef GRIPPER_SUPERVISOR_H
#define GRIPPER_SUPERVISOR_H

#include <kdl/tree.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/frames.hpp>

#include "nasa_r2_common_msgs/GripperPositionState.h"
#include "nasa_r2_common_msgs/StringArray.h"
#include "nasa_r2_common_msgs/GripperEnvironment.h"
#include "nasa_r2_common_msgs/GripperPositionCommand.h"
#include "nasa_r2_common_msgs/JointControlDataArray.h"
#include "actionlib_msgs/GoalStatusArray.h"
#include "nasa_robodyn_utilities/BaseGripperConverter.h"

struct EndEffectorData
{
    bool locked;
    bool loaded;
    bool verified;
};

enum GripperSupervisorStates
{
	IDLE=0,
	FAULT, 
	FAILURE,
	LOCK,
	RELEASE,
	SET, 
	PENDING, 
	SET_PENDING
};

class GripperSupervisor : public BaseGripperConverter
{
public:
    GripperSupervisor();
    ~GripperSupervisor();
    
    // intitialize
    void InitializeMaps(const std::vector<std::string>& grippers);

	// put data into maps
    void PopulateGripperStateInfo(const nasa_r2_common_msgs::GripperPositionState& eefState); 
    void PopulateBaseFrameInfo(const nasa_r2_common_msgs::StringArray& baseFrames);
    void PopulateEnvironmentInfo(const nasa_r2_common_msgs::GripperEnvironment& gripEnv);
    void PopulateGoalStatus(const actionlib_msgs::GoalStatusArray& goalStats);
    void PopulateGripperCommandsIn(const nasa_r2_common_msgs::GripperPositionCommand& gripCmd);
    void PopulateJointControlMode(const nasa_r2_common_msgs::JointControlDataArray& jntMode);
    
    // create messages from data
    void PopulateJointControlCommandMsg(nasa_r2_common_msgs::JointControlDataArray& modeCmdOut);
    void PopulateGoalStatusMsg(actionlib_msgs::GoalStatusArray& statusOut);
    void PopulateGripperCommandMsg(nasa_r2_common_msgs::GripperPositionCommand& cmdOut);
    
    // do the work
    void CheckTransitions(bool baseOverride);  // take the argument out!
    
    // access the data (mostly for unit testing)
    int getState(const std::string& name);
    
    // set limits (from component properties)
    void setLimits(const double& lowDuty, const double& highDuty);
    
    std::map<std::string, EndEffectorData> endEffectorMap;
    std::vector<std::string> baseFrameList;
    std::map<std::string, int> gripperEnvMap;
    std::map<std::string, int> gripperJointModeMap;
    nasa_r2_common_msgs::GripperPositionCommand gripperCmds;
    actionlib_msgs::GoalStatusArray goalStatuses;

private:
    
    actionlib_msgs::GoalStatusArray goalStatusesOut;
    nasa_r2_common_msgs::GripperPositionCommand gripperCmdOut;
    nasa_r2_common_msgs::JointControlDataArray jointModeCommands;
    
    std::map<std::string, bool> errorStates; // 1- error, 0- no error
    std::map<std::string, GripperSupervisorStates> stateMap;
    //std::map<std::string, bool> cmdSent;
    
    std::string setpoint;
    double motcomLimit;
    double hiDutyCycleLimit, loDutyCycleLimit;
    
    int detectChangeInErrorState(const std::string& name);
    int isReleaseValid(const std::string& name);
    
    int transToFault(const std::string& name);
    int transToIdle(const std::string& name);
    int transToFailure(const std::string& name);
    int transToLock(const std::string& name);
    int transToRelease(const std::string& name);
    int transToSet(const std::string& name);
    
    int runStateMachine(const std::string& name);
    
};

#endif
