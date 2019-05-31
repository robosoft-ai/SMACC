/*****************************************************************************************************************
 * ReelRobotix Inc. - Software License Agreement      Copyright (c) 2018
 * 	 Authors: Pablo Inigo Blasco, Brett Aldrich
 *
 ******************************************************************************************************************/
#pragma once
#include "smacc/smacc_state_machine.h"
#include "smacc/component.h"

namespace smacc
{

//#define SMACC_STATE_BEHAVIOR(BEHAVIOR_CLASS) \
//    SmaccStateBehavior* definesBehavioralSmaccState() \
//    {                                                 \
//      BEHAVIOR_CLASS* behavior;                         \
//      this->requiresComponent(behavior);              \
//      return behavior;                                \
//    }                                                 

#define SMACC_STATE_BEHAVIOR(BEHAVIOR_NAME) \
    SmaccStateBehavior* definesBehavioralSmaccState() \
    {                                                 \
      ROS_INFO("trying to get the substate behavior: %s",BEHAVIOR_NAME); \
      SmaccStateBehavior* behavior;                         \
      bool found = this->getGlobalSMData(BEHAVIOR_NAME, behavior);              \
      ROS_INFO("substate behavior '%s' exists? %d",BEHAVIOR_NAME, found);        \
      return behavior;                                \
    }                                                 

class SmaccStateBehavior: public smacc::ISmaccComponent
{ 
    
    public:
    // hapens when
    // return true to destroy the object and false to keep it alive 

    ISmaccStateMachine* stateMachine; 

    template <typename SmaccComponentType>
    void requiresComponent(SmaccComponentType*& storage, ros::NodeHandle nh=ros::NodeHandle(), std::string value="")
    {
      stateMachine->requiresComponent(storage,nh, value);
    }

    virtual void onEntry()
    {
      ROS_INFO("SmaccStateBehavior onEntry");
    }

    virtual bool onExit()
    {
      ROS_INFO("SmaccStateBehavior onExit");
      return true;
    }  
};


template< class MostDerived,
          class Context,
          class InnerInitial = mpl::list<>,
          sc::history_mode historyMode = sc::has_no_history>
class SmaccState : public sc::simple_state<
  MostDerived, Context, InnerInitial, historyMode >
{
  typedef sc::simple_state< MostDerived, Context, InnerInitial, historyMode >
    base_type;

  public:
    ros::NodeHandle nh;

    // by default nullptr
    SmaccStateBehavior* stateBehavior;

    //////////////////////////////////////////////////////////////////////////
    struct my_context
    {
      my_context( typename base_type::context_ptr_type pContext ) :
        pContext_( pContext )
      {
      }

      typename base_type::context_ptr_type pContext_;
    };
    
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
    bool param(std::string param_name, T& param_val, const T& default_val) const
    {
        return nh.param(param_name, param_val, default_val);
    }
    
  
    typedef SmaccState my_base;


    template <typename T>
    bool getGlobalSMData(std::string name, T& ret)
    {
        return base_type::outermost_context().getGlobalSMData(name,ret);
    }

    // store globally in this state machine. (By value parameter )
    template <typename T>
    void setGlobalSMData(std::string name, T value)
    {
        base_type::outermost_context().setGlobalSMData(name,value);
    }

    template <typename SmaccComponentType>
    void requiresComponent(SmaccComponentType*& storage, ros::NodeHandle nh=ros::NodeHandle(), std::string value="")
    {
      base_type::outermost_context().requiresComponent(storage,nh, value);
    }

    ISmaccStateMachine& getStateMachine()
    {
      return base_type::outermost_context();
    }

    SmaccState() = delete;
    
    // constructor that initialize the state ros node handle 
    SmaccState( my_context ctx )
    {
      this->set_context( ctx.pContext_ );

      ros::NodeHandle contextNh = optionalNodeHandle(ctx.pContext_);

      ROS_DEBUG("context node handle namespace: %s", contextNh.getNamespace().c_str());
      if(contextNh.getNamespace() == "/" )
      {
        contextNh = ros::NodeHandle(cleanTypeName(typeid(Context)));
      }

      std::string classname = cleanTypeName(typeid(MostDerived));

      this->nh = ros::NodeHandle(contextNh.getNamespace() + std::string("/")+ classname );
    
      ROS_DEBUG("nodehandle namespace: %s", nh.getNamespace().c_str());
      MostDerived* test;

      this->updateCurrentState(true,test); //<MostDerived>

      this->setParam("created", true);
      stateBehavior = static_cast<MostDerived*>(this)->definesBehavioralSmaccState();

      if(stateBehavior !=nullptr)
      {
        ROS_INFO("Behavioral State");
        stateBehavior->stateMachine = &this->getStateMachine();
        stateBehavior->onEntry();
      }
      else
      {
        ROS_INFO("Not behavioral State");
        static_cast<MostDerived*>(this)->onEntry();
      }
    }

    template <typename StateType>
    void updateCurrentState(bool active, StateType* test)
    {
      base_type::outermost_context().updateCurrentState(active, test);
    }

    InnerInitial* smacc_inner_type;
 
    virtual ~SmaccState() 
    {
      ROS_ERROR("exiting state");

      if(this->stateBehavior!=nullptr)
      {
        stateBehavior->onExit();
      }
      else
      {
        //this->updateCurrentState<MostDerived>(false);
        static_cast<MostDerived*>(this)->onExit();
      }
    }

  public:

    SmaccStateBehavior* definesBehavioralSmaccState()
    {
      return nullptr;
    }

    // this method is static-polimorphic because of the curiously recurreing pattern. It
    // calls to the most derived class onEntry method if declared on smacc state construction
    void onEntry()
    {

    }

  // this method is static-polimorphic because of the curiously recurreing pattern. It
    // calls to the most derived class onExit method if declared on smacc state destruction
    void onExit()
    {

    }

    //////////////////////////////////////////////////////////////////////////
    // The following declarations should be private.
    // They are only public because many compilers lack template friends.
    //////////////////////////////////////////////////////////////////////////
    // See base class for documentation
    typedef typename base_type::outermost_context_base_type
      outermost_context_base_type;
    typedef typename base_type::inner_context_ptr_type inner_context_ptr_type;
    typedef typename base_type::context_ptr_type context_ptr_type;
    typedef typename base_type::inner_initial_list inner_initial_list;

    static void initial_deep_construct(
      outermost_context_base_type & outermostContextBase )
    {
      deep_construct( &outermostContextBase, outermostContextBase );
    }

    // See base class for documentation
    static void deep_construct(
      const context_ptr_type & pContext,
      outermost_context_base_type & outermostContextBase )
    {
      const inner_context_ptr_type pInnerContext(
        shallow_construct( pContext, outermostContextBase ) );
      base_type::template deep_construct_inner< inner_initial_list >(
        pInnerContext, outermostContextBase );
    }

    static inner_context_ptr_type shallow_construct(
      const context_ptr_type & pContext,
      outermost_context_base_type & outermostContextBase )
    {
      const inner_context_ptr_type pInnerContext(
        new MostDerived( my_context( pContext ) ) );
      outermostContextBase.add( pInnerContext );
      return pInnerContext;
    }
};
}
