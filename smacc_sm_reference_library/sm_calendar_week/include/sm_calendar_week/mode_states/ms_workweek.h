#include <smacc/smacc.h>
namespace sm_calendar_week
{
// STATE DECLARATION
class MsWorkweek : public smacc::SmaccState<MsWorkweek, SmCalendarWeek, StMonday>
{
public:
   using SmaccState::SmaccState;
};
} // namespace sm_calendar_week
