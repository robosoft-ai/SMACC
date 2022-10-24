#include <smacc/smacc.h>
namespace sm_calendar_week
{
// STATE DECLARATION
class MsWeekend : public smacc::SmaccState<MsWeekend, SmCalendarWeek, StSaturday>
{
public:
   using SmaccState::SmaccState;

// TRANSITION TABLE
 //  typedef mpl::list<

  // >reactions;
};
} // namespace sm_calendar_week
