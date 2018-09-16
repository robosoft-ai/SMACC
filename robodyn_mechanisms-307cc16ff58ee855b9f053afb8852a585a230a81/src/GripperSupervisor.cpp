#include "nasa_robodyn_mechanisms_core/GripperSupervisor.h"
#include <iostream>

GripperSupervisor::GripperSupervisor(): setpoint("null"), motcomLimit(30)
{
}

GripperSupervisor::~GripperSupervisor()
{
}

void GripperSupervisor::InitializeMaps(const std::vector<std::string>& grippers)
{
    // Initialize zeros in the right ways
    EndEffectorData zeroData;
    zeroData.locked = false;
    zeroData.loaded = false;
    zeroData.verified = false;

    for(unsigned int i=0; i<grippers.size(); i++)
    {
        endEffectorMap[grippers[i]] = zeroData;
        gripperEnvMap[grippers[i]] = nasa_r2_common_msgs::GripperEnvironment::NONE;
        errorStates[grippers[i]] = false;
        stateMap[grippers[i]] = IDLE;
        gripperJointModeMap[grippers[i]] = nasa_r2_common_msgs::JointControlMode::IGNORE; 
    }
    
    return;

}

void GripperSupervisor::PopulateGripperStateInfo(const nasa_r2_common_msgs::GripperPositionState& eefState)
{
    unsigned int neef = eefState.name.size();

    // Iterate through all end effectors in message
    for(unsigned int i = 0; i<neef; i++)
    {
        if(endEffectorMap.count(eefState.name[i]) > 0)
        {
            endEffectorMap[eefState.name[i]].locked = eefState.locked[i];
            endEffectorMap[eefState.name[i]].loaded = eefState.loaded[i];
        }
    }

    return;

}
 
void GripperSupervisor::PopulateBaseFrameInfo(const nasa_r2_common_msgs::StringArray& baseFrames)
{
    baseFrameList.clear();
    for(unsigned int i=0; i<baseFrames.data.size(); i++)
    {
        baseFrameList.push_back(baseFrames.data[i]);
    }
    
    return;
}

void GripperSupervisor::PopulateEnvironmentInfo(const nasa_r2_common_msgs::GripperEnvironment& gripEnv)
{
    for(unsigned int i=0; i<gripEnv.name.size(); i++)
    {
        if(gripperEnvMap.count(gripEnv.name[i]) > 0)
        {
            gripperEnvMap[gripEnv.name[i]] = gripEnv.environment[i];
        }
    }
    
    return;
}

void GripperSupervisor::PopulateGoalStatus(const actionlib_msgs::GoalStatusArray& goalStats)
{
    for(unsigned int i=0; i<goalStats.status_list.size(); i++)
    {
        // clear this vector after using it in transitions
        if(goalStats.status_list[i].status == actionlib_msgs::GoalStatus::ABORTED ||
           goalStats.status_list[i].status == actionlib_msgs::GoalStatus::SUCCEEDED)
        {
            goalStatuses.status_list.push_back(goalStats.status_list[i]);
        }
    }
    
    return;
}

void GripperSupervisor::PopulateGripperCommandsIn(const nasa_r2_common_msgs::GripperPositionCommand& gripCmd)
{
    // clear this vector after using it in transitions
    gripperCmds = gripCmd;
    
    actionlib_msgs::GoalStatus gStat;
    gStat.goal_id.stamp = gripperCmds.header.stamp;
    gStat.status = actionlib_msgs::GoalStatus::PENDING;
    gStat.text = "Command pending";
    
    for(unsigned int i=0; i<gripperCmds.name.size(); i++)
    {
        gStat.goal_id.id = gripperCmds.name[i];
        goalStatusesOut.status_list.push_back(gStat);
    }
    
    return;
}

void GripperSupervisor::PopulateJointControlMode(const nasa_r2_common_msgs::JointControlDataArray& jntMode)
{
    for(unsigned int i=0; i<jntMode.joint.size(); i++)
    {
        if(gripperJointModeMap.count(jntMode.joint[i]) > 0)
            gripperJointModeMap[jntMode.joint[i]] = jntMode.data[i].controlMode.state;
    }
    
    return;
}
    
void GripperSupervisor::PopulateJointControlCommandMsg(nasa_r2_common_msgs::JointControlDataArray& modeCmdOut)
{
    modeCmdOut = jointModeCommands;
    modeCmdOut.header.stamp = ros::Time::now();
    
    jointModeCommands.joint.clear();
    jointModeCommands.data.clear();
    return;
}

void GripperSupervisor::PopulateGoalStatusMsg(actionlib_msgs::GoalStatusArray& statusOut)
{
    statusOut = goalStatusesOut;
    goalStatusesOut.status_list.clear();
    return;
}

void GripperSupervisor::PopulateGripperCommandMsg(nasa_r2_common_msgs::GripperPositionCommand& cmdOut)
{
    // put all commands into one message
    cmdOut = gripperCmdOut;
    cmdOut.header.stamp = ros::Time::now();
    
    gripperCmdOut.name.clear();
    gripperCmdOut.setpoint.clear();
    gripperCmdOut.command.clear();
    gripperCmdOut.dutyCycle.clear();
    
    return;
}

void GripperSupervisor::CheckTransitions(bool baseOverride)
{
    // first, check to see if there are changes in fault bits
    std::map<std::string, bool>::const_iterator it = errorStates.begin();
    while( it != errorStates.end())
    {
        int change = detectChangeInErrorState(it->first);
        if(change && it->second)
            transToFault(it->first);
        else if(change && !it->second)
            transToIdle(it->first);
            
        it++;
    }
    
    // next, add transitions for goal statuses
    for(unsigned int i=0; i<goalStatuses.status_list.size(); i++)
    {
        std::string name = goalStatuses.status_list[i].goal_id.id;
        if(stateMap.count(name) > 0) 
        {
            if(gripperJointModeMap[name] != nasa_r2_common_msgs::JointControlMode::FAULTED)
            {
                if(goalStatuses.status_list[i].status == actionlib_msgs::GoalStatus::ABORTED)
                {
                    transToFailure(name);
                    
                    // send warning via status message
                    actionlib_msgs::GoalStatus gStat;
                    gStat.goal_id.stamp = gripperCmds.header.stamp;
                    gStat.goal_id.id = name;
                    gStat.status = actionlib_msgs::GoalStatus::ABORTED;
                    gStat.text = "Transition to failure";
                    goalStatusesOut.status_list.push_back(gStat);       
                }
                else 
                {
                    transToIdle(name);
                    
                    // send warning via status message
                    actionlib_msgs::GoalStatus gStat;
                    gStat.goal_id.stamp = gripperCmds.header.stamp;
                    gStat.goal_id.id = name;
                    gStat.status = actionlib_msgs::GoalStatus::SUCCEEDED;
                    gStat.text = "Command succeeded";
                    goalStatusesOut.status_list.push_back(gStat);       
                }
            }
        }
        runStateMachine(name);
    }
    goalStatuses.status_list.clear();
    
    // finally, run through all gripper commands
    for(unsigned int i=0; i<gripperCmds.name.size(); i++)
    {
        std::string name = gripperCmds.name[i];
        if(stateMap.count(name) > 0 && gripperEnvMap.count(name) > 0)
        {
            if(gripperCmds.command[i] == "cancel")
            {
                // add command to cancel this gripper
                gripperCmdOut.name.push_back(gripperCmds.name[i]);
                gripperCmdOut.setpoint.push_back(gripperCmds.setpoint[i]); 
                gripperCmdOut.command.push_back("c_cancel"); 
                gripperCmdOut.dutyCycle.push_back(hiDutyCycleLimit);
                
                actionlib_msgs::GoalStatus gStat;
                gStat.goal_id.stamp = gripperCmds.header.stamp;
                gStat.goal_id.id = gripperCmds.name[i];
                gStat.status = actionlib_msgs::GoalStatus::ACTIVE;
                gStat.text = "Sending cancel";
                goalStatusesOut.status_list.push_back(gStat);   
            
                if(!(stateMap[name] == FAULT or stateMap[name] == FAILURE))
                    transToIdle(name);
            }
            else if(gripperCmds.command[i] == "reset")
            {
                if(stateMap[name] == FAILURE)
                    transToIdle(name);
            }
            else if(gripperCmds.command[i] == "lock")
            {
                if(!(stateMap[name] == FAULT or stateMap[name] == FAILURE) && gripperEnvMap[name] != nasa_r2_common_msgs::GripperEnvironment::NONE)
                {
                    setpoint = gripperCmds.setpoint[i];
                    motcomLimit = hiDutyCycleLimit;
                    transToLock(name);
                }
                else
                {
                    // send warning via status message
                    actionlib_msgs::GoalStatus gStat;
                    gStat.goal_id.stamp = gripperCmds.header.stamp;
                    gStat.goal_id.id = name;
                    gStat.status = actionlib_msgs::GoalStatus::REJECTED;
                    gStat.text = "Lock not valid";
                    goalStatusesOut.status_list.push_back(gStat);               
                }
            }
            else if(gripperCmds.command[i] == "release")
            {
                if(!(stateMap[name] == FAULT or stateMap[name] == FAILURE) && (isReleaseValid(name) || baseOverride))
                {
                    setpoint = gripperCmds.setpoint[i];
                    motcomLimit = hiDutyCycleLimit;
                    transToRelease(name);
                }
                else
                {
                    // send warning via status message
                    actionlib_msgs::GoalStatus gStat;
                    gStat.goal_id.stamp = gripperCmds.header.stamp;
                    gStat.goal_id.id = name;
                    gStat.status = actionlib_msgs::GoalStatus::REJECTED;
                    gStat.text = "Release not valid";
                    goalStatusesOut.status_list.push_back(gStat);
                }
            }
            else if(gripperCmds.command[i] == "set")
            {
                // including "backdoor" to allow gripper to close regardless of other base frames being present
                if(!(stateMap[name] == FAULT or stateMap[name] == FAILURE) && (isReleaseValid(name) || baseOverride))
                {
                    setpoint = gripperCmds.setpoint[i];
                    if(gripperEnvMap[name] == nasa_r2_common_msgs::GripperEnvironment::FREESPACE)
                    {
                        motcomLimit = loDutyCycleLimit;
                        transToSet(name);
                    }
                    else if(gripperEnvMap[name] != nasa_r2_common_msgs::GripperEnvironment::NONE)
                    {
                        motcomLimit = hiDutyCycleLimit;
                        transToSet(name);
                    } else
                    {
                        // send warning via status message
                        actionlib_msgs::GoalStatus gStat;
                        gStat.goal_id.stamp = gripperCmds.header.stamp;
                        gStat.goal_id.id = name;
                        gStat.status = actionlib_msgs::GoalStatus::REJECTED;
                        gStat.text = "Environment is unconfirmed";
                        goalStatusesOut.status_list.push_back(gStat);
                    }
                } else
                { 
                    // send warning via status message
                    actionlib_msgs::GoalStatus gStat;
                    gStat.goal_id.stamp = gripperCmds.header.stamp;
                    gStat.goal_id.id = name;
                    gStat.status = actionlib_msgs::GoalStatus::REJECTED;
                    gStat.text = "Set not valid";
                    goalStatusesOut.status_list.push_back(gStat);
                }
            }
            else
            {
                // send warning via status message
                actionlib_msgs::GoalStatus gStat;
                gStat.goal_id.stamp = gripperCmds.header.stamp;
                gStat.goal_id.id = name;
                gStat.status = actionlib_msgs::GoalStatus::REJECTED;
                gStat.text = "Not a valid command";
                goalStatusesOut.status_list.push_back(gStat);
            }
        }
        
        runStateMachine(name);
    }
    gripperCmds.name.clear();
    gripperCmds.setpoint.clear();
    gripperCmds.command.clear();
    gripperCmds.dutyCycle.clear();
    
    it = errorStates.begin();
    while(it != errorStates.end())
    {
        runStateMachine(it->first); 
        it++;
    }       
    
    return;
}

int GripperSupervisor::detectChangeInErrorState(const std::string& name)
{
    
    if(errorStates.count(name) == 0 || gripperJointModeMap.count(name) == 0)
        return -1;
    
    int mode = gripperJointModeMap[name];
    if ( (mode == nasa_r2_common_msgs::JointControlMode::FAULTED and !errorStates[name]) || (mode != nasa_r2_common_msgs::JointControlMode::FAULTED and errorStates[name]))
    {
        if(mode == nasa_r2_common_msgs::JointControlMode::FAULTED)
            errorStates[name] = true;
        else
            errorStates[name] = false;
            
        return 1; // true, there is a change!
    }
    
    return 0;  // no change
}

int GripperSupervisor::isReleaseValid(const std::string& name)
{
    // check base frames
    int matches = 0;
    unsigned int numBaseFrames = baseFrameList.size();
    std::vector<std::string> baseToGripperList;
    
    for(unsigned int i=0; i<numBaseFrames; i++)
    {
        baseToGripperList.push_back(BaseFrameToGripper(baseFrameList[i]));
        if(name == baseToGripperList[i])
            matches++;
    }
    
    if(numBaseFrames - matches <= 0)
        return 0; // not valid
        
    // Check if other base frame is not in drive or faulted
    for(unsigned int i=0; i<numBaseFrames; i++)
    {
        if(baseToGripperList[i] != name)
        {
            std::string oppositeFrame = FindOpposite(name);
            if( !oppositeFrame.empty())
            {
				std::map<std::string, int>::const_iterator it = gripperJointModeMap.find(oppositeFrame);
				if( it != gripperJointModeMap.end())
				{
					if( it->second != nasa_r2_common_msgs::JointControlMode::FAULTED || it->second != nasa_r2_common_msgs::JointControlMode::DRIVE)
						return 1; // valid
				}
            }
        }
    }
    
    return 0;  // not valid
}

int GripperSupervisor::transToFault(const std::string& name)
{
    if(stateMap.count(name) == 0)
        return -1;
        
    // add a command to park gripper
    nasa_r2_common_msgs::JointControlData jcd;
    jcd.controlMode.state = nasa_r2_common_msgs::JointControlMode::PARK;
    jointModeCommands.joint.push_back(name);
    jointModeCommands.data.push_back(jcd);
    
    // add command to cancel this gripper
    gripperCmdOut.name.push_back(name);
    gripperCmdOut.setpoint.push_back(setpoint); 
    gripperCmdOut.command.push_back("c_cancel");
    gripperCmdOut.dutyCycle.push_back(0);
    
    actionlib_msgs::GoalStatus gStat;
    gStat.goal_id.stamp = gripperCmds.header.stamp;
    gStat.goal_id.id = name;
    gStat.status = actionlib_msgs::GoalStatus::ABORTED;
    gStat.text = "Sending cancel due to gripper fault";
    goalStatusesOut.status_list.push_back(gStat);
    
    stateMap[name] = FAULT;
    
    return 0;
}
    
int GripperSupervisor::transToIdle(const std::string& name)
{
    if(stateMap.count(name) == 0)
        return -1;
        
    // add a command to park gripper
    if(stateMap[name] != SET_PENDING)
    {
        nasa_r2_common_msgs::JointControlData jcd;
        jcd.controlMode.state = nasa_r2_common_msgs::JointControlMode::PARK;
        jointModeCommands.joint.push_back(name);
        jointModeCommands.data.push_back(jcd);
    }
    
    stateMap[name] = IDLE;
    
    return 0;
}

int GripperSupervisor::transToFailure(const std::string& name)
{
    if(stateMap.count(name) == 0)
        return -1;
        
    // add a command to park gripper
    nasa_r2_common_msgs::JointControlData jcd;
    jcd.controlMode.state = nasa_r2_common_msgs::JointControlMode::PARK;
    jointModeCommands.joint.push_back(name);
    jointModeCommands.data.push_back(jcd);
    
    stateMap[name] = FAILURE;
    
    return 0;
}

int GripperSupervisor::transToLock(const std::string& name)
{
    if(stateMap.count(name) == 0 || gripperJointModeMap.count(name) == 0)
        return -1;
        
    // add a command to servo gripper
    if(!(gripperJointModeMap[name] == nasa_r2_common_msgs::JointControlMode::FAULTED  || 
        gripperJointModeMap[name] == nasa_r2_common_msgs::JointControlMode::DRIVE))
    {
        nasa_r2_common_msgs::JointControlData jcd;
        jcd.controlMode.state = nasa_r2_common_msgs::JointControlMode::DRIVE;
        jointModeCommands.joint.push_back(name);
        jointModeCommands.data.push_back(jcd);
    }
    
    stateMap[name] = LOCK;
    
    return 0;
}

int GripperSupervisor::transToRelease(const std::string& name)
{
    if(stateMap.count(name) == 0 || gripperJointModeMap.count(name) == 0)
        return -1;
    
    // add a command to servo gripper
    if(!(gripperJointModeMap[name] == nasa_r2_common_msgs::JointControlMode::FAULTED  || 
        gripperJointModeMap[name] == nasa_r2_common_msgs::JointControlMode::DRIVE))
    {
        nasa_r2_common_msgs::JointControlData jcd;
        jcd.controlMode.state = nasa_r2_common_msgs::JointControlMode::DRIVE;
        jointModeCommands.joint.push_back(name);
        jointModeCommands.data.push_back(jcd);
    }
    
    stateMap[name] = RELEASE;
    
    return 0;
}

int GripperSupervisor::transToSet(const std::string& name)
{
    if(stateMap.count(name) == 0 || gripperJointModeMap.count(name) == 0)
        return -1;
        
    // add a command to servo gripper
    if(!(gripperJointModeMap[name] == nasa_r2_common_msgs::JointControlMode::FAULTED  || 
        gripperJointModeMap[name] == nasa_r2_common_msgs::JointControlMode::DRIVE))
    {
        nasa_r2_common_msgs::JointControlData jcd;
        jcd.controlMode.state = nasa_r2_common_msgs::JointControlMode::DRIVE;
        jointModeCommands.joint.push_back(name);
        jointModeCommands.data.push_back(jcd);
    }
    
    stateMap[name] = SET;
    
    return 0;
}

int GripperSupervisor::runStateMachine(const std::string& name)
{
    if(gripperJointModeMap.count(name) == 0 || stateMap.count(name) == 0)
        return -1;
    
    switch(stateMap[name])
    {
        case IDLE:
        case FAULT:
        case FAILURE:
        case PENDING:
        case SET_PENDING:
            break;
        
        case LOCK:
            if( gripperJointModeMap[name] == nasa_r2_common_msgs::JointControlMode::DRIVE)
            {
                // send the lock command
                gripperCmdOut.name.push_back(name);
                gripperCmdOut.setpoint.push_back(setpoint); 
                gripperCmdOut.command.push_back("c_lock"); 
                gripperCmdOut.dutyCycle.push_back(motcomLimit);
                
                actionlib_msgs::GoalStatus gStat;
                gStat.goal_id.stamp = gripperCmds.header.stamp;
                gStat.goal_id.id = name;
                gStat.status = actionlib_msgs::GoalStatus::ACTIVE;
                gStat.text = "Sending lock command";
                goalStatusesOut.status_list.push_back(gStat);
                
                stateMap[name] = PENDING;
            }
            break;
            
        case RELEASE:
            if(gripperJointModeMap[name] == nasa_r2_common_msgs::JointControlMode::DRIVE)
            {
                // send the release command
                gripperCmdOut.name.push_back(name);
                gripperCmdOut.setpoint.push_back(setpoint); 
                gripperCmdOut.command.push_back("c_release");
                gripperCmdOut.dutyCycle.push_back(motcomLimit);
                
                actionlib_msgs::GoalStatus gStat;
                gStat.goal_id.stamp = gripperCmds.header.stamp;
                gStat.goal_id.id = name;
                gStat.status = actionlib_msgs::GoalStatus::ACTIVE;
                gStat.text = "Sending release command";
                goalStatusesOut.status_list.push_back(gStat);
                
                stateMap[name] = PENDING;
            }
            break;
            
        case SET:
            if(gripperJointModeMap[name] == nasa_r2_common_msgs::JointControlMode::DRIVE)
            {
                // send the set command
                gripperCmdOut.name.push_back(name);
                gripperCmdOut.setpoint.push_back(setpoint); 
                gripperCmdOut.command.push_back("c_set");
                gripperCmdOut.dutyCycle.push_back(motcomLimit);
                
                actionlib_msgs::GoalStatus gStat;
                gStat.goal_id.stamp = gripperCmds.header.stamp;
                gStat.goal_id.id = name;
                gStat.status = actionlib_msgs::GoalStatus::ACTIVE;
                gStat.text = "Sending set command";
                goalStatusesOut.status_list.push_back(gStat);
                
                stateMap[name] = SET_PENDING;
            }
            break;
            
        default:
            // unknown
            break;
    }
    
    return 0;
}

int GripperSupervisor::getState(const std::string& name)
{
    if(stateMap.count(name) == 0)
        return -1;
        
    return stateMap[name];
}

void GripperSupervisor::setLimits(const double& lowDuty, const double& highDuty)
{
    loDutyCycleLimit = lowDuty;
    hiDutyCycleLimit = highDuty;
}
