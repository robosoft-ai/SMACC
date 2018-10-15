
#pragma once

#include <ros/ros.h>
#include <boost/statechart/state.hpp>
#include <boost/statechart/simple_state.hpp>
#include <boost/statechart/event.hpp>
#include <boost/statechart/fifo_scheduler.hpp>
#include <boost/statechart/asynchronous_state_machine.hpp>
#include <boost/statechart/custom_reaction.hpp>

#include <boost/config.hpp>
#include <boost/intrusive_ptr.hpp>
#include <boost/mpl/list.hpp>
#include <boost/function.hpp>
#include <boost/signals2.hpp>

#include <boost/mpl/list.hpp>
#include <actionlib/client/simple_action_client.h>

#include <boost/statechart/termination.hpp>
#include <boost/statechart/transition.hpp>

namespace sc = boost::statechart;

using namespace boost;

typedef sc::fifo_scheduler<> SmaccScheduler;

typedef std::allocator< void > SmaccAllocator;

//typedef boost::fast_pool_allocator< int > MyAllocator;

//template <typename MostDerived, typename Context, typename InnerList= mpl::list<>>
//using SmaccState = sc::state<MostDerived,Context,InnerList>;

template< class MostDerived,
          class Context,
          class InnerInitial = mpl::list<>,
          sc::history_mode historyMode = sc::has_no_history >
class SmaccState : public sc::simple_state<
  MostDerived, Context, InnerInitial, historyMode >
{
  typedef sc::simple_state< MostDerived, Context, InnerInitial, historyMode >
    base_type;

  protected:
    //////////////////////////////////////////////////////////////////////////
    struct my_context
    {
      my_context( typename base_type::context_ptr_type pContext ) :
        pContext_( pContext )
      {
      }

      typename base_type::context_ptr_type pContext_;
    };

    typedef SmaccState my_base;

    SmaccState( my_context ctx )
    {
      this->set_context( ctx.pContext_ );
    }

    ~SmaccState() {}

  public:
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

//------------------------------------------------------------

typedef boost::statechart::processor_container<boost::statechart::fifo_scheduler<>, boost::function0<void>, std::allocator<void> >::processor_context my_context;

namespace smacc
{
  class ISmaccStateMachine;
  class ISmaccActionClient;
  class SignalDetector;

  struct EvActionClientSuccess : sc::event< EvActionClientSuccess > 
  {
      smacc::ISmaccActionClient* client;

      actionlib::SimpleClientGoalState getResult() const;
  };
}


