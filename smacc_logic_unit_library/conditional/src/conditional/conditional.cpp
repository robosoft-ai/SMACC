#include <conditional/lu_conditional.h>

namespace smacc
{

LuConditional::~LuConditional()
{

}


bool LuConditional::triggers()
{
    return this->conditionFlag;
}
}