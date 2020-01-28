#include <smacc/smacc.h>
namespace sm_packML
{
// STATE DECLARATION
class MsRecover : public smacc::SmaccState<MsRecover, SmThreesome>
{
public:
   using SmaccState::SmaccState;

// TRANSITION TABLE
   typedef mpl::list<
    
   Transition<EvToDeep, sc::deep_history<StState1>, SUCCESS>
   
   >reactions;
};
} // namespace sm_packML