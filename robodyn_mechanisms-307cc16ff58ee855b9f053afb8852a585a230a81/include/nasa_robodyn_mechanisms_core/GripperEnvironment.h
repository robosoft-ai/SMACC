#ifndef GRIPPER_ENVIRONMENT_H
#define GRIPPER_ENVIRONMENT_H

#include <kdl/tree.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/frames.hpp>

#include "nasa_r2_common_msgs/GripperPositionState.h"
#include "nasa_r2_common_msgs/StringArray.h"
#include "nasa_r2_common_msgs/GripperEnvironment.h"
#include "actionlib_msgs/GoalStatusArray.h"
#include "nasa_robodyn_utilities/BaseGripperConverter.h"
#include "nasa_robodyn_utilities/RosMsgConverter.h"


class GripperEnvironment : public BaseGripperConverter
{
public:
    GripperEnvironment();
    ~GripperEnvironment();
    
    /// intitialize
    void initializeMap(const std::vector<std::string>& grippers);
    void setComponentNames(const std::string& handrail, const std::string& seattrack);

	/// put data into maps
    void populateGripperStateInfo(const nasa_r2_common_msgs::GripperPositionState& eefState); 
    void populateBaseFrameInfo(const nasa_r2_common_msgs::StringArray& baseFrames);
    void populateGoalStatus(const actionlib_msgs::GoalStatusArray& goalStats);
    void populatePoseStateInfo(const nasa_r2_common_msgs::PoseState& poseCmd);
    void setUserCommand(const std::string gripperName, const int& cmd);
    
    /// create messages from data
    void createGripperEnvironmentMsg(nasa_r2_common_msgs::GripperEnvironment& gripEnv);
    
    /// do the work
    void update();
	
    struct GripperEnvInfo
    {
		int environment;
		int userCommand;
		int gripperState;
		int highForceSubState;
		KDL::Frame frame;
		KDL::Frame prevFrame;
	};
	
	/// for unit testing
	bool getMapEntry(const std::string& name, GripperEnvInfo& info);
	std::string getBaseFrame();
	int getGoalStatusArraySize();

private:
    std::string handrailComponent, seattrackComponent, baseFrame;
    std::map<std::string, GripperEnvInfo>::iterator envIt;
    std::vector<std::string>::iterator nameIt;
    std::vector<std::string> baseFrameList;
    std::map<std::string, GripperEnvInfo> gripperEnvMap;
    actionlib_msgs::GoalStatusArray goalStatuses;
    KDL::Frame tempFrame;
    std::string name;
    KDL::Twist frameDiff;
    int type;
    
    void transToHighForce(const int& attachType, std::map<std::string, GripperEnvInfo>::iterator& gripIt);
    void transToFreespace(std::map<std::string, GripperEnvInfo>::iterator& gripIt);
    bool checkForHighForce(std::map<std::string, GripperEnvInfo>::iterator& gripIt, int& attachType);
    bool checkForFreespaceUserCmd(std::map<std::string, GripperEnvInfo>::iterator& gripIt);
    bool checkForHighForceUserCmd(std::map<std::string, GripperEnvInfo>::iterator& gripIt, int& attachType);
    bool checkGoalStatus(const std::string& gripper, int& attachType);
    bool checkForMovement(const std::map<std::string, GripperEnvInfo>::iterator& gripIt);
    bool isGripperLocked(const std::map<std::string, GripperEnvInfo>::iterator& gripIt);
    bool isBaseFrame(const std::map<std::string, GripperEnvInfo>::iterator& gripIt);
    
    int findAttachType(const std::string& input);
};

#endif
