#include <smacc/smacc_state.h>

namespace smacc
{
std::string ISmaccState::getClassName()
{
    return demangleSymbol(typeid(*this).name());
}
}