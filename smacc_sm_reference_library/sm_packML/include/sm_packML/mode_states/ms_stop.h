#include <smacc/smacc.h>
namespace sm_packML
{
// STATE DECLARATION
class MsStop : public smacc::SmaccState<MsStop, SmPackML, StStopping>
{
public:
   using SmaccState::SmaccState;

// TRANSITION TABLE
 //  typedef mpl::list<
    
  // Transition<EvToDeep, sc::deep_history<StIdle>, SUCCESS>
   
  // >reactions;
};
} // namespace sm_packML