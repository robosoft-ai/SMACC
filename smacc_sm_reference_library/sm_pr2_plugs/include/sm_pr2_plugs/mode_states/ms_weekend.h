#include <smacc/smacc.h>
namespace sm_pr2_plugs{
// STATE DECLARATION
class MsWeekend : public smacc::SmaccState<MsWeekend, SmPR2Plugs, StSaturday>
{
public:
   using SmaccState::SmaccState;

// TRANSITION TABLE
 //  typedef mpl::list<
    
  // Transition<EvToDeep, sc::deep_history<StFriday>, SUCCESS>
   
  // >reactions;
};
} // namespace sm_pr2_plugs