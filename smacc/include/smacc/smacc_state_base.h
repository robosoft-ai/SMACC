#pragma once
#include <smacc/smacc_state.h>
#include <typeindex>
#include <typeinfo>

namespace smacc
{

template <class MostDerived,
          class Context,
          class InnerInitial = mpl::list<>,
          sc::history_mode historyMode = sc::has_no_history>
class SmaccState : public sc::simple_state<
                       MostDerived, Context, InnerInitial, historyMode>,
                   public ISmaccState
{
  typedef sc::simple_state<MostDerived, Context, InnerInitial, historyMode>
      base_type;

public:
  //////////////////////////////////////////////////////////////////////////
  struct my_context
  {
    my_context(typename base_type::context_ptr_type pContext) : pContext_(pContext)
    {
    }

    typename base_type::context_ptr_type pContext_;
  };

  typedef SmaccState my_base;

  template <typename T>
  bool getGlobalSMData(std::string name, T &ret)
  {
    return base_type::outermost_context().getGlobalSMData(name, ret);
  }

  // Store globally in this state machine. (By value parameter )
  template <typename T>
  void setGlobalSMData(std::string name, T value)
  {
    base_type::outermost_context().setGlobalSMData(name, value);
  }

  template <typename SmaccComponentType>
  void requiresComponent(SmaccComponentType *&storage, ros::NodeHandle nh = ros::NodeHandle(), std::string value = "", bool verbose = false)
  {
    base_type::outermost_context().requiresComponent(storage, nh, value, verbose);
  }

  virtual ISmaccStateMachine &getStateMachine()
  {
    return base_type::outermost_context();
  }

  SmaccState() = delete;

  // Constructor that initializes the state ros node handle
  SmaccState(my_context ctx)
  {
    ROS_WARN_STREAM("creatingState state: " << demangleSymbol(typeid(MostDerived).name()).c_str());

    this->set_context(ctx.pContext_);

    ros::NodeHandle contextNh = optionalNodeHandle(ctx.pContext_);

    ROS_DEBUG("context node handle namespace: %s", contextNh.getNamespace().c_str());
    if (contextNh.getNamespace() == "/")
    {
      contextNh = ros::NodeHandle(cleanShortTypeName(typeid(Context)));
    }

    std::string classname = cleanShortTypeName(typeid(MostDerived));

    this->nh = ros::NodeHandle(contextNh.getNamespace() + std::string("/") + classname);

    ROS_DEBUG("nodehandle namespace: %s", nh.getNamespace().c_str());
    MostDerived *test;

    this->updateCurrentState(true, test); //<MostDerived>

    this->setParam("created", true);

    // before dynamic onInitialize, we execute the onDefinition behavior configurations
    {
      ROS_INFO("-- STATIC STATE DESCRIPTION --");

      for (const auto &stateBehaviorsVector : SmaccStateInfo::staticBehaviorInfo)
      {
        ROS_INFO_STREAM(" - state info: " << demangleSymbol(stateBehaviorsVector.first->name()));
        for (auto &bhinfo : *(stateBehaviorsVector.second))
        {
          ROS_INFO_STREAM(" - substate behavior: " << demangleSymbol(bhinfo.behaviorType->name()));
        }
      }

      const std::type_info *tindex = &(typeid(MostDerived));
      auto &staticDefinedBehaviors = SmaccStateInfo::staticBehaviorInfo[tindex];

      ROS_INFO("---- BEGIN STATIC STATE ACTIONS");
      for (auto &bhinfo : *staticDefinedBehaviors)
      {
        ROS_INFO("- Creating static substate behavior");
        bhinfo.factoryFunction(this);
      }
      ROS_INFO("---- END STATIC STATE ACTIONS");
    }

    static_cast<MostDerived *>(this)->onInitialize();

    this->getStateMachine().notifyOnStateEntry(this);

    //ROS_INFO("Not behavioral State");
    static_cast<MostDerived *>(this)->onEntry();
  }

  template <typename StateType>
  void updateCurrentState(bool active, StateType *test)
  {
    base_type::outermost_context().updateCurrentState(active, test);
  }

  InnerInitial *smacc_inner_type;

  std::string getFullPathName()
  {
    auto smInfo = this->getStateMachine().info_;

    auto key = typeid(MostDerived).name();
    if (smInfo->states.count(key))
    {
      return smInfo->states[key]->getFullPath();
    }
    else
    {
      return "UnknownFullPath";
    }
  }

  std::string getFullName()
  {
    return demangleSymbol(typeid(MostDerived).name());
  }

  std::string getShortName()
  {
    return cleanShortTypeName(typeid(MostDerived));
  }

  virtual ~SmaccState()
  {
    auto fullname = demangleSymbol(typeid(MostDerived).name());
    ROS_WARN_STREAM("exiting state: " << fullname);

    this->getStateMachine().notifyOnStateExit(this);

    //this->updateCurrentState<MostDerived>(false);
    static_cast<MostDerived *>(this)->onExit();

    ROS_INFO_STREAM("throwing finish event " << fullname);
    this->throwFinishEvent();
    ROS_WARN_STREAM("state exit: " << fullname);
  }

  template <typename EventType>
  void postEvent(const EventType &ev)
  {
    getStateMachine().postEvent(ev);
  }

  void throwFinishEvent()
  {
    auto *finishEvent = new EvStateFinish<MostDerived>();
    finishEvent->state = static_cast<MostDerived *>(this);
    this->postEvent(finishEvent);
  }

public:
  // This method is static-polymorphic because of the curiously recurring template pattern. It
  // calls to the most derived class onEntry method if declared on smacc state construction
  void onInitialize()
  {
  }

  // This method is static-polymorphic because of the curiously recurring template pattern. It
  // calls to the most derived class onEntry method if declared on smacc state construction
  void onEntry()
  {
  }

  // this method is static-polimorphic because of the curiously recurreing pattern. It
  // calls to the most derived class onExit method if declared on smacc state destruction
  void onExit()
  {
  }

  template <typename TOrthogonal, typename TBehavior, typename ...Args>
  static void static_configure(Args && ...args)
  {
    auto strorthogonal = demangleSymbol(typeid(TOrthogonal).name());
    auto strbehavior = demangleSymbol(typeid(TBehavior).name());

    ROS_INFO_STREAM("Orthogonal " << strorthogonal << " -> " << strbehavior);

    StateBehaviorInfoEntry bhinfo;
    bhinfo.factoryFunction = [&](ISmaccState *state) {
      auto bh = std::make_shared<TBehavior>(std::forward<Args>(args)...);
      state->configure<TOrthogonal>(bh);
    };

    const std::type_info *tindex = &(typeid(MostDerived));
    bhinfo.behaviorType = &(typeid(TBehavior));
    bhinfo.orthogonalType = &(typeid(TOrthogonal));

    if (!SmaccStateInfo::staticBehaviorInfo.count(tindex))
      SmaccStateInfo::staticBehaviorInfo[tindex] = std::make_shared<std::vector<StateBehaviorInfoEntry>>();

    SmaccStateInfo::staticBehaviorInfo[tindex]->push_back(bhinfo);
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
      outermost_context_base_type &outermostContextBase)
  {
    deep_construct(&outermostContextBase, outermostContextBase);
  }

  // See base class for documentation
  static void deep_construct(
      const context_ptr_type &pContext,
      outermost_context_base_type &outermostContextBase)
  {
    const inner_context_ptr_type pInnerContext(
        shallow_construct(pContext, outermostContextBase));
    base_type::template deep_construct_inner<inner_initial_list>(
        pInnerContext, outermostContextBase);
  }

  static inner_context_ptr_type shallow_construct(
      const context_ptr_type &pContext,
      outermost_context_base_type &outermostContextBase)
  {
    const inner_context_ptr_type pInnerContext(
        new MostDerived(my_context(pContext)));
    outermostContextBase.add(pInnerContext);
    return pInnerContext;
  }
};
} // namespace smacc