/*****************************************************************************************************************
 * ReelRobotix Inc. - Software License Agreement      Copyright (c) 2018
 * 	 Authors: Pablo Inigo Blasco, Brett Aldrich
 *
 ******************************************************************************************************************/
#pragma once
#include <smacc/common.h>
#include <smacc/smacc_state_base.h>
#include <smacc/smacc_state_info.h>
#include <smacc/smacc_state_machine_info.h>

#include <smacc_msgs/SmaccContainerStructure.h>
#include <smacc_msgs/SmaccContainerInitialStatusCmd.h>
#include <smacc_msgs/SmaccContainerStatus.h>
//-------------------------------------------------------------------------------------------------

namespace smacc
{

/// State Machine
template <typename DerivedStateMachine, typename InitialStateType>
struct SmaccStateMachineBase : public ISmaccStateMachine,  public sc::asynchronous_state_machine<DerivedStateMachine, InitialStateType, SmaccScheduler, SmaccAllocator >
{
public:
    // The node handle for this state
    ros::NodeHandle nh;
    
    ros::Timer timer_;
    ros::Publisher stateMachineStructurePub_;
    ros::Publisher stateMachineStatePub_;

    //std::vector<std::shared_ptr<SmaccStateInfo>> currentState_;

    virtual void Reset()
    {
        ISmaccStateMachine::Reset();
        this->terminate();
        smacc::run<DerivedStateMachine>();
    }

    virtual void Stop()
    {
        ISmaccStateMachine::Stop();
        this->terminate();
    }

    virtual void EStop()
    {
        ISmaccStateMachine::EStop();
        this->terminate();
    }  
    
    SmaccStateMachineBase( my_context ctx, SignalDetector* signalDetector)
        :ISmaccStateMachine(signalDetector),
        sc::asynchronous_state_machine<DerivedStateMachine, InitialStateType, SmaccScheduler, SmaccAllocator >(ctx)
    {
        ROS_ERROR("State machine base creation");
        nh = ros::NodeHandle(cleanShortTypeName(typeid(DerivedStateMachine)));

        info_ = std::make_shared<SmaccStateMachineInfo>();
        info_->buildStateMachineInfo<InitialStateType>();
        
	    InitialStateType* test;
        updateCurrentState<InitialStateType>(true,test);
        
        info_->printAllStates();
          
        timer_= nh.createTimer(ros::Duration(0.1),&SmaccStateMachineBase<DerivedStateMachine,InitialStateType>::state_machine_visualization, this);
        stateMachineStructurePub_=nh.advertise<smacc_msgs::SmaccContainerStructure>("/RadialMotionStateMachine/smacc/container_structure",1);
        stateMachineStatePub_ = nh.advertise<smacc_msgs::SmaccContainerStatus>("/RadialMotionStateMachine/smacc/container_status",1);     
    }
    
    virtual ~SmaccStateMachineBase( )
    {
        //updateCurrentState<InitialStateType>(false);
    }

    // This function is defined in the Player.cpp
    virtual void initiate_impl() override
    {
        ROS_INFO("initiate_impl");
        sc::state_machine< DerivedStateMachine, InitialStateType, SmaccAllocator >::initiate();
    }

     // Delegates to ROS param access with the current NodeHandle
    template <typename T>
    bool getParam(std::string param_name, T& param_storage)
    {
        return nh.getParam(param_name, param_storage);
    }

    // Delegates to ROS param access with the current NodeHandle
    template <typename T>
    void setParam(std::string param_name, T param_val)
    {
        return nh.setParam(param_name, param_val);
    }

    // Delegates to ROS param access with the current NodeHandle
    template<typename T>
    bool param(std::string param_name, T& param_val, const T& default_val) const
    {
        return nh.param(param_name, param_val, default_val);
    }

    void createStructureMessage(std::shared_ptr<SmaccStateInfo> container, std::string currentPath, std::vector<smacc_msgs::SmaccContainerStructure>& structure_msgs, int deep)
    {
        if(deep ==0)
        {
            return;
        }

        smacc_msgs::SmaccContainerStructure structure_msg;
        structure_msg.header.stamp = ros::Time::now();

        std::string currentpathprefix;
        
        // ROOT STATE
        if(container==nullptr)
        {
            // THE NAME OF THE STATE MACHINE WITH A PREFIX '/' CHARACTER
            currentpathprefix = "/" + currentPath;
            structure_msg.path= currentpathprefix;
        }
        else // IT IS A SUBSTATE
        {
            if(currentPath!="")
                currentpathprefix = currentPath + "/"  ;
            else
                currentpathprefix = "/";

            structure_msg.path = currentpathprefix + container->demangledStateName;
        }

        static int i =0;
        for(auto& val: info_->states)
        {
            // childstate
            auto state = val.second;
            std::stringstream ss;
            if(state->parentState_== container)
            {
                structure_msg.children.push_back(state->toShortName());    

                for(auto& transition: state->transitions_)
                {
                    structure_msg.internal_outcomes.push_back("success");//transition.first);  
                    structure_msg.outcomes_to.push_back(transition.second->toShortName());
                    structure_msg.outcomes_from.push_back(state->toShortName());
                }

                if(state->parentState_!=nullptr)
                {
                    structure_msg.outcomes_to.push_back("success");
                    structure_msg.outcomes_from.push_back(state->toShortName());
                    structure_msg.container_outcomes.push_back("success");
                }
                /*
                if(state->transitions_.size() > 0)
                    structure_msg.container_outcomes.push_back(state->transitions_.begin()->first); 
                else if( state->parentState_!=nullptr && state->parentState_->transitions_.size()>0)
                    structure_msg.container_outcomes.push_back(state->parentState_->transitions_.begin()->first); 

                */
                
                createStructureMessage(state, structure_msg.path, structure_msgs, deep -1);
            }
        }

        structure_msgs.push_back(structure_msg);
    }

    void recursivePublishStatus(std::shared_ptr<SmaccStateInfo> parentstate)
    {
        //ROS_WARN_STREAM("*" + parentstate->toShortName());
        smacc_msgs::SmaccContainerStatus status_msg;
        status_msg.header.stamp = ros::Time::now();

        if(parentstate==nullptr)
        {
            status_msg.path = "/RadialMotion";

            for(auto& val: info_->states)
            {
                auto state = val.second;

                if(state->parentState_!=nullptr // only root elements
                    || !state->active_ )
                        continue;     

                //ROS_WARN_STREAM("active root state: " << state->toShortName());
                status_msg.active_states.push_back(state->toShortName());
            }

            status_msg.info = "HEART BEAT";
    
            status_msg.local_data.resize(6);
            status_msg.local_data[0] = 0x80;
            status_msg.local_data[1] = 0x02;
            status_msg.local_data[2] = 0x7d;
            status_msg.local_data[3] = 0x71;
            status_msg.local_data[4] = 0x00;
            status_msg.local_data[5] = 0x2e;

            stateMachineStatePub_.publish(status_msg);

            for(auto& val: info_->states)
            {
                auto state = val.second;

                if(state->parentState_!=nullptr // only root elements
                    || !state->active_ )
                        continue;     

                recursivePublishStatus(state);
            }


        }
        else
        {
            status_msg.path = "/RadialMotion/"+ parentstate->getFullPath();

            //status_msg.initial_states.push_back("NavigateToRadialStart");
            for(auto& child: parentstate->children_)
            {
                if(child->active_)
                {
                    //ROS_WARN_STREAM("active child state ("<< parentstate->toShortName()<<"): " << child->toShortName());
                    status_msg.active_states.push_back(child->toShortName());
                }
            }

             status_msg.info = "HEART BEAT";
        
            status_msg.local_data.resize(6);
            status_msg.local_data[0] = 0x80;
            status_msg.local_data[1] = 0x02;
            status_msg.local_data[2] = 0x7d;
            status_msg.local_data[3] = 0x71;
            status_msg.local_data[4] = 0x00;
            status_msg.local_data[5] = 0x2e;


            stateMachineStatePub_.publish(status_msg);

            for(auto& child: parentstate->children_)
            {
                if(child->active_ && !child->children_.empty())
                {
                    recursivePublishStatus(child);
                }
            }
        }
    }

    void state_machine_visualization(const ros::TimerEvent&)
    {
    /*
    header: 
    seq: 24
    stamp: 
        secs: 1545148095
        nsecs: 148165941
    frame_id: ''
    path: "/SM_ROOT"
    children: [FOO, BAR]
    internal_outcomes: [outcome1, outcome2, outcome2]
    outcomes_from: [FOO, FOO, BAR]
    outcomes_to: [BAR, outcome4, FOO]
    container_outcomes: [outcome4, outcome5]
    */

    std::vector<smacc_msgs::SmaccContainerStructure> structure_msgs;
    createStructureMessage(nullptr, "RadialMotion", structure_msgs,-1);

    /*
    structure_msg.children.push_back("NavigateToEndPoint");
    structure_msg.children.push_back("NavigateToRadialStart");
    structure_msg.children.push_back("ReturnToRadialStart");
    structure_msg.children.push_back("RotateDegress");         
    */

    /*
    structure_msg.container_outcomes.push_back("ev1");
    structure_msg.container_outcomes.push_back("ev2");
    structure_msg.container_outcomes.push_back("ev3");
    structure_msg.container_outcomes.push_back("ev4");
    */
    
    /*structure_msg.internal_outcomes.push_back("ev1");
    structure_msg.internal_outcomes.push_back("ev2");
    structure_msg.internal_outcomes.push_back("ev2");
    structure_msg.internal_outcomes.push_back("ev2");

    structure_msg.outcomes_from.push_back("NavigateToEndPoint");
    structure_msg.outcomes_from.push_back("NavigateToRadialStart");
    structure_msg.outcomes_from.push_back("ReturnToRadialStart");
    structure_msg.outcomes_from.push_back("RotateDegress");

    structure_msg.outcomes_to.push_back("ReturnToRadialStart");
    structure_msg.outcomes_to.push_back("NavigateToEndPoint");
    structure_msg.outcomes_to.push_back("RotateDegress");
    structure_msg.outcomes_to.push_back("NavigateToEndPoint");
*/
    for(int i= structure_msgs.size() -1 ; i >= 0 ; i--)
    {
        for(auto& structure_msg: structure_msgs)
        {
            stateMachineStructurePub_.publish(structure_msgs[i]);
        }
    }

    
    recursivePublishStatus(nullptr);
    

    /*    
    if(currentState_ !=nullptr && currentState_->parentState_==nullptr)
    {        
        smacc_msgs::SmaccContainerStatus status_msg;
        status_msg.header.stamp = ros::Time::now();

        status_msg.path = "/RadialMotion";
        status_msg.initial_states.push_back("NavigateToRadialStart");
        status_msg.active_states.push_back(currentState_->toShortName());
        status_msg.info = "HEART BEAT";
        
        status_msg.local_data.resize(6);
        status_msg.local_data[0] = 0x80;
        status_msg.local_data[1] = 0x02;
        status_msg.local_data[2] = 0x7d;
        status_msg.local_data[3] =  0x71;
        status_msg.local_data[4] = 0x00;
        status_msg.local_data[5] = 0x2e;


        stateMachineStatePub_.publish(status_msg);
    }*/

//


    /*
    ---
    header: 
    seq: 316
    stamp: 
        secs: 1545154196
        nsecs: 289685964
    frame_id: ''
    path: "/SM_ROOT"
    initial_states: [FOO]
    active_states: [None]
    local_data: !!binary |
    gAJ9cQAu
    info: "HEART BEAT"
    */
    
}      
};
}
