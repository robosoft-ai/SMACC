#include <conditional/lu_conditional.h>

namespace smacc
{
namespace logic_units
{
LuConditional::~LuConditional()
{
}

bool LuConditional::triggers()
{
    return this->conditionFlag;
}
} // namespace logic_units
} // namespace smacc