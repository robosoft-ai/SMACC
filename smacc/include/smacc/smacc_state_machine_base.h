/*****************************************************************************************************************
 * ReelRobotix Inc. - Software License Agreement      Copyright (c) 2018
 * 	 Authors: Pablo Inigo Blasco, Brett Aldrich
 *
 ******************************************************************************************************************/
#pragma once
#include <smacc/common.h>
#include <smacc/smacc_state.h>
#include <type_traits>
#include <boost/mpl/for_each.hpp>
#include <boost/mpl/list.hpp>
#include <boost/utility/enable_if.hpp>
#include <smach_msgs/SmachContainerStructure.h>
#include <smach_msgs/SmachContainerInitialStatusCmd.h>
#include <smach_msgs/SmachContainerStatus.h>
//-------------------------------------------------------------------------------------------------

namespace smacc
{

class SmaccStateMachineInfo;


//--------------------------------
class SmaccStateInfo: public std::enable_shared_from_this<SmaccStateInfo>
{
public:
    bool active_;
    std::string fullStateName;
    std::string demangledStateName;

    std::shared_ptr<SmaccStateMachineInfo> stateMachine_;
    std::shared_ptr<SmaccStateInfo> parentState_;
    std::map<std::string,std::shared_ptr<SmaccStateInfo>> transitions_;

    std::vector<std::shared_ptr<SmaccStateInfo>> children_;
    int depth_;

    SmaccStateInfo(std::shared_ptr<SmaccStateInfo> parentState, std::shared_ptr<SmaccStateMachineInfo> stateMachineInfo)
    {
        parentState_ = parentState;
        stateMachine_ = stateMachineInfo;

        if(parentState_!=nullptr)
            depth_ = parentState->depth_ +1;
    }

    inline int depth() const{return depth_;}


    std::string getFullPath()
    {
        if( parentState_==nullptr)
            return this->toShortName();
        else
            return this->parentState_->getFullPath() + "/" + this->toShortName();
    }

    template <typename StateType>
    std::shared_ptr<SmaccStateInfo> createChildState();

    template <typename EvType>
    void declareTransition(std::shared_ptr<SmaccStateInfo>& dstState);

    const std::string& toShortName() const
    {
        return this->demangledStateName;
    }
};

//--------------------------------
class SmaccStateMachineInfo: public std::enable_shared_from_this<SmaccStateMachineInfo>
{
public:
    std::map<std::string,std::shared_ptr<SmaccStateInfo>> states;

    template <typename InitialStateType>
    void buildStateMachineInfo();

    template <typename StateType>
    std::shared_ptr<SmaccStateInfo> createState(std::shared_ptr<SmaccStateInfo> parentState);

    template <typename StateType>
    bool containsState()
    {
        auto typeNameStr = typeid(StateType).name();

        return states.count(typeNameStr)>0;
    }

    template <typename StateType>
    std::shared_ptr<SmaccStateInfo> getState()
    {
        if( this->containsState<StateType>())
        {
            return states[typeid(StateType).name()];
        }
        return nullptr;
    }

    template <typename StateType>
    void addState(std::shared_ptr<StateType>& state);


    void printAllStates()
    {
        for(auto& val: this->states)
        {
            auto state = val.second;

            std::stringstream ss;
            if(state->parentState_== nullptr)
            {
                ss << "**** Print StateType: " << state->demangledStateName <<std::endl;
                ss << " Childstates:" << std::endl;

                for(auto& child: state->children_)
                {
                    ss << " - " << child->demangledStateName << std::endl;
                }

                ss << " Transitions:" << std::endl;

                for(auto& transition: state->transitions_)
                {
                    ss << " - " << transition.first << " -> " << transition.second->demangledStateName << std::endl;
                }

                ROS_INFO_STREAM(ss.str());
            }
        }
    }
};
//--------------------------------


template<typename T>
struct type_
{
    using type= T;
};

template <typename T>
struct add_type_wrapper
{
    using type = type_<T>;
};
//---------------------------------------------

struct AddSubState
{
    std::shared_ptr<SmaccStateInfo>& parentState_;
    AddSubState(std::shared_ptr<SmaccStateInfo>& parentState)
    : parentState_(parentState)
    {
    }

    template <typename T> 
    void operator()(T);
};

struct AddTransition
{
    std::shared_ptr<SmaccStateInfo>& currentState_;
    
    AddTransition(std::shared_ptr<SmaccStateInfo>& currentState)
    : currentState_(currentState)
    {
    }

    template <typename T>
    void operator()(T);
};

template <typename InitialStateType>
struct WalkStatesExecutor
{
    static void walkStates(std::shared_ptr<SmaccStateInfo>& currentState, bool rootInitialNode);
};

template <typename T> 
void AddSubState::operator()(T)
{
    using type_t = typename T::type;
    //auto childState = this->parentState_->createChildState<type_t>()
    WalkStatesExecutor<type_t>::walkStates(parentState_, false);   
}
//--------------------------------------------
template<typename T>
typename disable_if<boost::mpl::is_sequence<T>>::type
processSubState(std::shared_ptr<SmaccStateInfo>& parentState)
{
    WalkStatesExecutor<T>::walkStates(parentState, false);
}

template<typename T>
typename enable_if<boost::mpl::is_sequence<T>>::type
processSubState(std::shared_ptr<SmaccStateInfo>& parentState)
{
    using boost::mpl::_1;
    using wrappedList = typename boost::mpl::transform<T,add_type_wrapper<_1>>::type;
    boost::mpl::for_each<wrappedList>(AddSubState(parentState));
}
//--------------------------------------------
template<typename T>
typename enable_if<boost::mpl::is_sequence<T>>::type
processTransitions(std::shared_ptr<SmaccStateInfo>& sourceState)
{
    using boost::mpl::_1;
    using wrappedList = typename boost::mpl::transform<T,add_type_wrapper<_1>>::type;
    boost::mpl::for_each<wrappedList>(AddTransition(sourceState));
}

template<typename Ev, typename Dst>
void processTransition(statechart::transition<Ev,Dst>* , std::shared_ptr<SmaccStateInfo>& sourceState)
{
    //ROS_INFO_STREAM("GOTCHA");

    if(!sourceState->stateMachine_->containsState<Dst>())
    {
        auto siblingnode = sourceState->stateMachine_->createState<Dst>(sourceState->parentState_);
        WalkStatesExecutor<Dst>::walkStates(siblingnode, true);
        sourceState->declareTransition< Ev>(siblingnode);
    }
    else
    {
        auto siblingnode = sourceState->stateMachine_->getState<Dst>();
        sourceState->declareTransition< Ev>(siblingnode);
    }
}

template <typename EvType>
void SmaccStateInfo::declareTransition(std::shared_ptr<SmaccStateInfo>& dstState)
{
    auto evtype = demangledTypeName<EvType>();
    transitions_[evtype] = dstState;
}

template<typename Ev>
void processTransition(statechart::custom_reaction<Ev>* , std::shared_ptr<SmaccStateInfo>& sourceState)
{
    //ROS_INFO_STREAM("GOTCHA");
}

template<typename T>
typename disable_if<boost::mpl::is_sequence<T>>::type
processTransitions(std::shared_ptr<SmaccStateInfo>& sourceState)
{
    //ROS_INFO_STREAM("state transition from: " << sourceState->demangledStateName << " of type: " << demangledTypeName<T>());
    T* dummy;
    processTransition(dummy,sourceState);


}
//--------------------------------------------
template <typename T> 
void AddTransition::operator()(T)
{
    using type_t = typename T::type;
    processTransitions<type_t>(currentState_);
}

template <typename InitialStateType>
  void WalkStatesExecutor<InitialStateType>::walkStates(std::shared_ptr<SmaccStateInfo>& parentState, bool rootInitialNode)
{
    //ros::Duration(1).sleep();
    auto currentname = demangledTypeName<InitialStateType>();

    std::shared_ptr<SmaccStateInfo> targetState;

    if(!rootInitialNode)
    {
        if(parentState->stateMachine_->containsState<InitialStateType>())
        {
            // it already exist: break;
            return;
        }
    
        targetState  = parentState->createChildState<InitialStateType>();
    }
    else
    {
        targetState  = parentState;
    }

    typedef typename std::remove_pointer<decltype(InitialStateType::smacc_inner_type)>::type InnerType;
    processSubState<InnerType>(targetState);

    // -------------------- REACTIONS --------------------
    typedef typename InitialStateType::reactions reactions;
    //ROS_INFO_STREAM("state machine initial state reactions: "<< demangledTypeName<reactions>());
   
    processTransitions<reactions>(targetState);
}

template <typename InitialStateType>
void SmaccStateMachineInfo::buildStateMachineInfo()
{
    auto initialState = this->createState<InitialStateType>(nullptr);
    WalkStatesExecutor<InitialStateType>::walkStates(initialState, true);
}

template <typename StateType>
std::shared_ptr<SmaccStateInfo> SmaccStateMachineInfo::createState(std::shared_ptr<SmaccStateInfo> parent)
{
    auto thisptr = this->shared_from_this();
    auto state = std::shared_ptr<SmaccStateInfo>(new SmaccStateInfo(parent, thisptr));
    state->demangledStateName = demangledTypeName<StateType>();
    state->fullStateName = typeid(StateType).name();

    std::vector<std::string> strs;
    boost::split(strs,state->demangledStateName,boost::is_any_of(":"));

    state->demangledStateName = strs.back();

    if(parent!=nullptr)
    {
        parent->children_.push_back(state);
    }

    this->addState(state);

    return state;
}

template <typename StateType>
void SmaccStateMachineInfo::addState(std::shared_ptr<StateType>& state)
{    
    states[state->fullStateName] = state;
}

template <typename StateType>
std::shared_ptr<SmaccStateInfo> SmaccStateInfo::createChildState()
{
    auto childState = this->stateMachine_->createState<StateType>(shared_from_this());
    //this->stateMachine_->addState(childState);
    //stateMachineInfo.addState(stateMachineInfo)
    //stateNames.push_back(currentname);
    //ROS_INFO("------------");
    //ROS_INFO_STREAM("** STATE state: "<< this->demangledStateName);

    return childState;
}


/// State Machine
template <typename DerivedStateMachine, typename InitialStateType>
struct SmaccStateMachineBase : public ISmaccStateMachine,  public sc::asynchronous_state_machine<DerivedStateMachine, InitialStateType, SmaccScheduler, SmaccAllocator >
{
public:
    // the node handle for this state
    ros::NodeHandle nh;
    std::shared_ptr<SmaccStateMachineInfo> info_;

    ros::Timer timer_;
    ros::Publisher stateMachineStructurePub_;
    ros::Publisher stateMachineStatePub_;

    //std::vector<std::shared_ptr<SmaccStateInfo>> currentState_;

    template <typename StateType>
    void updateCurrentState(bool active)
    {
        auto a =  info_->getState<StateType>();

        if(a!=nullptr)
        {
                ROS_WARN_STREAM("setting state active "<< active <<": " << a->getFullPath());
                a->active_ = active;
        }
        else
        {
            ROS_ERROR_STREAM("updated state not found: " <<typeid(StateType).name());
        }  

        
        /*  
        if(a !=nullptr && a->parentState_==nullptr)
        {
            currentState_ =a;
            ROS_ERROR("really updated");
        }*/
    }
  
    
    SmaccStateMachineBase( my_context ctx, SignalDetector* signalDetector)
        :ISmaccStateMachine(signalDetector),
        sc::asynchronous_state_machine<DerivedStateMachine, InitialStateType, SmaccScheduler, SmaccAllocator >(ctx)
    {
        nh = ros::NodeHandle(cleanTypeName(typeid(DerivedStateMachine)));

        info_ = std::make_shared<SmaccStateMachineInfo>();
        info_->buildStateMachineInfo<InitialStateType>();
        updateCurrentState<InitialStateType>(true);
        
        info_->printAllStates();

          
        timer_= nh.createTimer(ros::Duration(0.1),&SmaccStateMachineBase<DerivedStateMachine,InitialStateType>::state_machine_visualization, this);
        stateMachineStructurePub_=nh.advertise<smach_msgs::SmachContainerStructure>("/RadialMotionStateMachine/smach/container_structure",1);
        stateMachineStatePub_ = nh.advertise<smach_msgs::SmachContainerStatus>("/RadialMotionStateMachine/smach/container_status",1);     
    
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

     // delegates to ROS param access with the current NodeHandle
    template <typename T>
    bool getParam(std::string param_name, T& param_storage)
    {
        return nh.getParam(param_name, param_storage);
    }

    // delegates to ROS param access with the current NodeHandle
    template <typename T>
    void setParam(std::string param_name, T param_val)
    {
        return nh.setParam(param_name, param_val);
    }

    // delegates to ROS param access with the current NodeHandle
    template<typename T>
    bool param(std::string param_name, T& param_val, T default_val) const
    {
        return nh.param(param_name, param_val, default_val);
    }

    void createStructureMessage(std::shared_ptr<SmaccStateInfo> container, std::string currentPath, std::vector<smach_msgs::SmachContainerStructure>& structure_msgs, int deep)
    {
        if(deep ==0)
        {
            return;
        }

        smach_msgs::SmachContainerStructure structure_msg;
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
        smach_msgs::SmachContainerStatus status_msg;
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

    std::vector<smach_msgs::SmachContainerStructure> structure_msgs;
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

    
    //ROS_WARN("status");
    recursivePublishStatus(nullptr);
    

    /*    
    if(currentState_ !=nullptr && currentState_->parentState_==nullptr)
    {        
        smach_msgs::SmachContainerStatus status_msg;
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
