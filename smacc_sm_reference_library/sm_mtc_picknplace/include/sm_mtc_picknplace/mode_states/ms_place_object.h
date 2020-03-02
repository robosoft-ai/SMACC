#include <smacc/smacc.h>
namespace sm_mtc_picknplace
{
// STATE DECLARATION
class MsPlaceObject : public smacc::SmaccState<MsPlaceObject, SmMTCPickNPlace, StLowerObject, sc::has_full_history>
{
public:
   using SmaccState::SmaccState;

// TRANSITION TABLE
 //  typedef mpl::list<
    
  // Transition<EvToDeep, sc::deep_history<StMoveToHome>, SUCCESS>
   
  // >reactions;
};
} // namespace sm_mtc_picknplace