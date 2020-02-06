#include <smacc/smacc.h>
namespace sm_calendar_week
{
// STATE DECLARATION
class MsWorkweek : public smacc::SmaccState<MsWorkweek, SmCalendarWeek, StStarting, sc::has_full_history>
{
public:
   using SmaccState::SmaccState;
};
} // namespace sm_calendar_week