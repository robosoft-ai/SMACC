#include <smacc/client.h>
#include <smacc/impl/smacc_client_impl.h>

namespace smacc
{
ISmaccClient::ISmaccClient()
{
    
}

ISmaccClient::~ISmaccClient()
{
}

void ISmaccClient::initialize()
{
}

std::string ISmaccClient::getName() const
{
    std::string keyname = demangleSymbol(typeid(*this).name());
    return keyname;
}

void ISmaccClient::setStateMachine(ISmaccStateMachine* stateMachine)
{
    stateMachine_ = stateMachine;
}
}