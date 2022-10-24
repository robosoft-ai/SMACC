#include <smacc/smacc.h>
namespace sm_packml
{
// STATE DECLARATION
class MsStop : public smacc::SmaccState<MsStop, SmPackML, StStopping>
{
public:
   using SmaccState::SmaccState;

// TRANSITION TABLE
 //  typedef mpl::list<

  // >reactions;
};
} // namespace sm_packml
