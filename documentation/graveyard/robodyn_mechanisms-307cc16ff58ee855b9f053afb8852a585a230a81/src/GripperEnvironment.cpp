#include "nasa_robodyn_mechanisms_core/GripperEnvironment.h"

GripperEnvironment::GripperEnvironment()
{
}

GripperEnvironment::~GripperEnvironment()
{
}

void GripperEnvironment::initializeMap(const std::vector<std::string>& grippers)
{
    // Initialize zeros in the right ways
    GripperEnvInfo initData;
    
    initData.environment = nasa_r2_common_msgs::GripperEnvironment::NONE;
	initData.gripperState = 0;
	initData.userCommand = nasa_r2_common_msgs::GripperEnvironment::NONE;
	initData.highForceSubState = 0;
	initData.frame = KDL::Frame();
	initData.prevFrame = KDL::Frame();

    for(unsigned int i=0; i<grippers.size(); i++)
    {
        gripperEnvMap[grippers[i]] = initData;
    }

}

void GripperEnvironment::setComponentNames(const std::string& handrail, const std::string& seattrack)
{
	handrailComponent = handrail;
	seattrackComponent = seattrack;
}

bool GripperEnvironment::getMapEntry(const std::string& name, GripperEnvInfo& info)
{
	if((envIt = gripperEnvMap.find(name)) != gripperEnvMap.end())
	{
		info = envIt->second;
		return true;
	}
	
	return false;
}

std::string GripperEnvironment::getBaseFrame()
{
	if(!baseFrameList.empty())
	{
		return baseFrameList[0];
	}
	
	return "";
}

int GripperEnvironment::getGoalStatusArraySize()
{
	return goalStatuses.status_list.size();
}

void GripperEnvironment::setUserCommand(const std::string gripperName, const int& cmd)
{
	if((envIt = gripperEnvMap.find(gripperName)) != gripperEnvMap.end())
	{
		envIt->second.userCommand = cmd;
	}
}	

void GripperEnvironment::populateGripperStateInfo(const nasa_r2_common_msgs::GripperPositionState& gripperState)
{
    // Iterate through all end effectors in message
    for(unsigned int i = 0; i<gripperState.name.size(); i++)
    {
        if((envIt = gripperEnvMap.find(gripperState.name[i])) != gripperEnvMap.end())
        {
            if(gripperState.name.size() == gripperState.locked.size())
            {
				envIt->second.gripperState = gripperState.locked[i];
			}
        }
    }
}
 
void GripperEnvironment::populateBaseFrameInfo(const nasa_r2_common_msgs::StringArray& baseFrames)
{
    baseFrameList = baseFrames.data;
}

void GripperEnvironment::populatePoseStateInfo(const nasa_r2_common_msgs::PoseState& poseCmd)
{
	if(baseFrameList.empty())
	{
		return;
	}

	// iterate through map
	for(envIt = gripperEnvMap.begin(); envIt != gripperEnvMap.end(); envIt++)
	{
		try
		{
			RosMsgConverter::PoseStateToFrame(poseCmd, baseFrameList[0], GripperToBaseFrame(envIt->first), tempFrame);
		}
		catch(std::logic_error& e)
		{
			return;
		}
		envIt->second.prevFrame = envIt->second.frame;
		envIt->second.frame = tempFrame;
	}		
}

void GripperEnvironment::populateGoalStatus(const actionlib_msgs::GoalStatusArray& goalStats)
{
    goalStatuses.status_list.clear();
    if(findAttachType(goalStats.header.frame_id) == nasa_r2_common_msgs::GripperEnvironment::HANDRAIL || 
       findAttachType(goalStats.header.frame_id) == nasa_r2_common_msgs::GripperEnvironment::SEATTRACK)
    {
        for(unsigned int i=0; i<goalStats.status_list.size(); i++)
        {
            // clear this vector after using it in transitions
            if(goalStats.status_list[i].status == actionlib_msgs::GoalStatus::SUCCEEDED)
            {
                goalStatuses.status_list.push_back(goalStats.status_list[i]);
            }
        }
    }	
}

void GripperEnvironment::createGripperEnvironmentMsg(nasa_r2_common_msgs::GripperEnvironment& gripEnv)
{
    gripEnv.header.stamp = ros::Time::now();
    gripEnv.name.clear();
    gripEnv.environment.clear();
    
    for(envIt = gripperEnvMap.begin(); envIt != gripperEnvMap.end(); envIt++)
    {
		gripEnv.name.push_back(envIt->first);
		
		gripEnv.environment.push_back(envIt->second.environment);
	}
}

void GripperEnvironment::update()
{
	for(envIt = gripperEnvMap.begin(); envIt != gripperEnvMap.end(); envIt++)
	{
		switch(envIt->second.environment)
		{
			case nasa_r2_common_msgs::GripperEnvironment::NONE:
			    if(baseFrameList.size() > 0)
			    {
					if(isBaseFrame(envIt))
					{
					    transToHighForce(nasa_r2_common_msgs::GripperEnvironment::HANDRAIL, envIt);
					}
					else
					{
						transToFreespace(envIt);
					}
				}
			    
			    break;
			
			case nasa_r2_common_msgs::GripperEnvironment::FREESPACE:
			    if(checkForHighForce(envIt, type))
			    {
					transToHighForce(type, envIt);
				}
			    
			    break;
			
			case nasa_r2_common_msgs::GripperEnvironment::HANDRAIL:
			case nasa_r2_common_msgs::GripperEnvironment::SEATTRACK:
			    if(checkForFreespaceUserCmd(envIt))
			    {
					transToFreespace(envIt);
					break;
				}
			    if(envIt->second.highForceSubState == 0)
			    {
					if(checkForMovement(envIt))
					{
						transToFreespace(envIt);
					}
					else
					{
						if(isGripperLocked(envIt))
						{
							envIt->second.highForceSubState = 1;
						}
					}
				}
				else
				{
					if(!isGripperLocked(envIt))
					{
						transToFreespace(envIt);
					}
				}
			    
			    break;
			    
			default:
				break;
		}
	}
}

void GripperEnvironment::transToHighForce(const int& attachType, std::map<std::string, GripperEnvInfo>::iterator& gripIt)
{
	gripIt->second.environment = attachType;
	if(isGripperLocked(gripIt))
	{
		gripIt->second.highForceSubState = 1;
	}
	else
	{
		gripIt->second.highForceSubState = 0;
		if(baseFrameList.size() > 0)
		{
			baseFrame = baseFrameList[0];
		}
		else
		{
			baseFrame = "/r2/robot_world";
		}
	}
}

void GripperEnvironment::transToFreespace(std::map<std::string, GripperEnvInfo>::iterator& gripIt)
{
	gripIt->second.environment = nasa_r2_common_msgs::GripperEnvironment::FREESPACE;
	gripIt->second.highForceSubState = 0;
}

bool GripperEnvironment::checkForHighForce(std::map<std::string, GripperEnvInfo>::iterator& gripIt, int& attachType)
{
	if(checkForHighForceUserCmd(gripIt, attachType))
	{ 
		return true;
	}
	else if(checkGoalStatus(gripIt->first, attachType))
	{
		return true;
	}
	else
	{
		attachType = nasa_r2_common_msgs::GripperEnvironment::NONE;
		return false;
	}
}

bool GripperEnvironment::checkForFreespaceUserCmd(std::map<std::string, GripperEnvInfo>::iterator& gripIt)
{
	if(gripIt->second.userCommand == nasa_r2_common_msgs::GripperEnvironment::FREESPACE)
	{
		gripIt->second.userCommand = nasa_r2_common_msgs::GripperEnvironment::NONE;
		return true;
	}
	
	return false;
}

bool GripperEnvironment::checkForHighForceUserCmd(std::map<std::string, GripperEnvInfo>::iterator& gripIt, int& attachType)
{
	attachType = gripIt->second.userCommand;
	if(gripIt->second.userCommand == nasa_r2_common_msgs::GripperEnvironment::HANDRAIL || gripIt->second.userCommand == nasa_r2_common_msgs::GripperEnvironment::SEATTRACK)
	{
		gripIt->second.userCommand = nasa_r2_common_msgs::GripperEnvironment::NONE;
		return true;
	}
	else
	{
		return false;
	}
}

bool GripperEnvironment::checkGoalStatus(const std::string& gripper, int& attachType)
{
	attachType = nasa_r2_common_msgs::GripperEnvironment::NONE;
	for(unsigned int i=0; i<goalStatuses.status_list.size(); i++)
	{
		if(FindSide(goalStatuses.status_list[i].goal_id.id) == FindSide(gripper))
		{
			if((attachType = findAttachType(goalStatuses.status_list[i].goal_id.id)) == nasa_r2_common_msgs::GripperEnvironment::NONE)
			{
				return false;
			}
			goalStatuses.status_list.clear();
			return true;
		}
	}
	
	return false;	
}

bool GripperEnvironment::checkForMovement(const std::map<std::string, GripperEnvInfo>::iterator& gripIt)
{
	if(baseFrameList.size() > 0)
	{
		if(baseFrame != baseFrameList[0])
		{
			return true;
		}
	}
	else
	{
		return true;
	}
	
	/// Check if pose command changed
	frameDiff = KDL::diff(gripIt->second.frame, gripIt->second.prevFrame);
	for(unsigned int i=0; i<3; i++)
	{
		if(fabs(frameDiff.vel[i]) > 0.01 || fabs(frameDiff.rot[i]) > 0.01)
		{
			return true;
		}
	}
	
	return false;
}

bool GripperEnvironment::isGripperLocked(const std::map<std::string, GripperEnvInfo>::iterator& gripIt)
{
	if(gripIt->second.gripperState == 0)
	{
		return false;
	}
	else
	{
		return true;
	}
}

bool GripperEnvironment::isBaseFrame(const std::map<std::string, GripperEnvInfo>::iterator& gripIt)
{
	if(isGripperLocked(gripIt))
	{
		if(std::find(baseFrameList.begin(), baseFrameList.end(), GripperToBaseFrame(gripIt->first)) != baseFrameList.end())
		{
			return true;
		}
	}
	
	return false;
}

int GripperEnvironment::findAttachType(const std::string& input)
{
	std::size_t pos = input.find("Handrail");
	if( pos != std::string::npos)
	{
		return nasa_r2_common_msgs::GripperEnvironment::HANDRAIL;
	}
	else
	{
		pos = input.find("Seattrack");
		if( pos != std::string::npos)
		{
			return nasa_r2_common_msgs::GripperEnvironment::SEATTRACK;
		}
		else
		{
			return nasa_r2_common_msgs::GripperEnvironment::NONE;
		}
	}
	
	return nasa_r2_common_msgs::GripperEnvironment::NONE;
}
