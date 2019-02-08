/*****************************************************************************************************************
 * ReelRobotix Inc. - Software License Agreement      Copyright (c) 2018
 * 	 Authors: Pablo Inigo Blasco, Brett Aldrich
 *
 ******************************************************************************************************************/
#pragma once
#include "smacc/smacc_state_machine.h"

namespace smacc
{
template< class MostDerived,
          class Context,
          class InnerInitial = mpl::list<>,
          sc::history_mode historyMode = sc::has_no_history >
class SmaccState : public sc::simple_state<
  MostDerived, Context, InnerInitial, historyMode >
{
  typedef sc::simple_state< MostDerived, Context, InnerInitial, historyMode >
    base_type;

  public:
    ros::NodeHandle nh;

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

    template<typename T>
    bool param(std::string param_name, T& param_val, const T default_val) const
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
    void requiresComponent(SmaccComponentType*& storage, ros::NodeHandle nh=ros::NodeHandle())
    {
      base_type::outermost_context().requiresComponent(storage,nh);
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
      this->updateCurrentState<MostDerived>(true);

      this->setParam("created", true);
      static_cast<MostDerived*>(this)->onEntry();
    }

    template <typename StateType>
    void updateCurrentState(bool active)
    {
      base_type::outermost_context().updateCurrentState<StateType>(active);
    }

    InnerInitial* smacc_inner_type;
 
    virtual ~SmaccState() 
    {
      ROS_ERROR("exiting state");
      //this->updateCurrentState<MostDerived>(false);
      static_cast<MostDerived*>(this)->onExit();
    }

  public:

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