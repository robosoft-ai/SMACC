#include <smacc/logic_unit.h>

namespace smacc
{
template <typename TEv>
void LogicUnit::declarePostEvent(smacc::introspection::typelist<TEv>)
{
    this->postEventFn = [this]() {
        ROS_INFO_STREAM("[Logic Unit Base] postingfn posting event: " << demangleSymbol<TEv>());
        auto *ev = new TEv();
        this->ownerState->getStateMachine().postEvent(ev);
    };
}
} // namespace smacc