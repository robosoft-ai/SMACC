#include <boost/statechart/transition.hpp>

namespace smacc
{

struct default_transition_name {};

//////////////////////////////////////////////////////////////////////////////
template< class Event, 
          class Destination,
          typename Tag = default_transition_name,
          class TransitionContext = boost::statechart::detail::no_context< Event >,
          void ( TransitionContext::*pTransitionAction )( const Event & ) =
            &boost::statechart::detail::no_context< Event >::no_function > 
class transition: public sc::transition<Event,Destination,TransitionContext, pTransitionAction>
{
    public:
    typedef Tag TRANSITION_TAG;

//   private:
//     //////////////////////////////////////////////////////////////////////////
//     template< class State >
//     struct reactions
//     {
//       static result react_without_action( State & stt )
//       {
//         return stt.template transit< Destination >();
//       }

//       static result react_with_action( State & stt, const Event & evt )
//       {
//         return stt.template transit< Destination >( pTransitionAction, evt );
//       }
//     };

//   public:
//     //////////////////////////////////////////////////////////////////////////
//     // The following declarations should be private.
//     // They are only public because many compilers lack template friends.
//     //////////////////////////////////////////////////////////////////////////
//     template< class State, class EventBase, class IdType >
//     static detail::reaction_result react(
//       State & stt, const EventBase & evt, const IdType & eventType )
//     {
//       typedef detail::reaction_dispatcher<
//         reactions< State >, State, EventBase, Event, TransitionContext, IdType
//       > dispatcher;
//       return dispatcher::react( stt, evt, eventType );
//     }
};

}