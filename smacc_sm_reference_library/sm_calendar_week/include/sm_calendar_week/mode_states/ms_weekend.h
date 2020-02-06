#include <smacc/smacc.h>
namespace sm_calendar_week
{
// STATE DECLARATION
class MsWeekend : public smacc::SmaccState<MsWeekend, SmCalendarWeek, StStopping>
{
public:
   using SmaccState::SmaccState;

// TRANSITION TABLE
 //  typedef mpl::list<
    
  // Transition<EvToDeep, sc::deep_history<StIdle>, SUCCESS>
   
  // >reactions;
};
} // namespace sm_calendar_week