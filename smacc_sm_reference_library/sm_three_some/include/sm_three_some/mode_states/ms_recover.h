#include <smacc/smacc.h>
namespace sm_three_some
{
class MsRecover : public smacc::SmaccState<MsRecover, SmThreeSome>
{
public:
   using SmaccState::SmaccState;

   typedef mpl::list<
       smacc::Transition<EvToDeep, sc::deep_history<StState1>, SUCCESS>>
       reactions;
};
} // namespace sm_three_some