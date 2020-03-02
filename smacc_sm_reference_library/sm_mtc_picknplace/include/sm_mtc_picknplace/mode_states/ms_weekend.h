#include <smacc/smacc.h>
namespace sm_mtc_picknplace
{
// STATE DECLARATION
class MsWeekend : public smacc::SmaccState<MsWeekend, SmMTCPickNPlace, StSaturday>
{
public:
   using SmaccState::SmaccState;

// TRANSITION TABLE
 //  typedef mpl::list<
    
  // Transition<EvToDeep, sc::deep_history<StFriday>, SUCCESS>
   
  // >reactions;
};
} // namespace sm_mtc_picknplace