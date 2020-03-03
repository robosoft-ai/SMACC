#include <smacc/smacc.h>
namespace sm_mtc_pour
{
// STATE DECLARATION
class MsPlaceBottle : public smacc::SmaccState<MsPlaceBottle, SmMTCPour, StPutDownObject, sc::has_full_history>
{
public:
   using SmaccState::SmaccState;

// TRANSITION TABLE
 //  typedef mpl::list<
    
  // Transition<EvToDeep, sc::deep_history<StMoveHome>, SUCCESS>
   
  // >reactions;
};
} // namespace sm_mtc_pour