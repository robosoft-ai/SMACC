#pragma once
#include <smacc/smacc_orthogonal.h>
#include <smacc/smacc_client.h>

namespace smacc
{
class IEventFactory
{
    template <typename T>
    void postEvent()
    {
    }
};

template <typename TDerived, typename TObjectTag>
class EventFactory
{
    std::function<void()> postEvent;

    template <typename TComponent>
    void onComponentAdded(TComponent *t)
    {
        t->configureEventSourceTypes(this);
    }
};

template <typename TObjectTag, typename TClient, typename... TArgs>
std::shared_ptr<TClient> Orthogonal::createClient(TArgs... args)
{
    ROS_INFO("[%s] creates a client of type '%s' and object tag '%s'",
             demangleType(typeid(*this)).c_str(),
             demangledTypeName<TClient>().c_str(),
             demangledTypeName<TObjectTag>().c_str());
    auto client = std::make_shared<TClient>(args...);
    client->setStateMachine(stateMachine_);

    client->template configureEventSourceTypes<TClient, TObjectTag>();
    EventFactory<TClient, TObjectTag> eventfactory;

    clients_.push_back(client);
    return client;
}

template <typename SmaccComponentType>
void Orthogonal::requiresComponent(SmaccComponentType *&storage)
{
    if (stateMachine_ == nullptr)
    {
        ROS_ERROR("Cannot use the requiresComponent funcionality from an orthogonal before onInitialize");
    }
    else
    {
        stateMachine_->requiresComponent(storage);
    }
}

template <typename SmaccClientType>
void Orthogonal::requiresClient(SmaccClientType *&storage)
{
    for (auto &client : clients_)
    {
        storage = dynamic_cast<SmaccClientType *>(client.get());
        if (storage != nullptr)
            return;
    }

    ROS_ERROR("[Client requirement] Client of type '%s' not found in any orthogonal of the current state machine. This may produce a segmentation fault in some posterior code.", demangleSymbol<SmaccClientType>().c_str());
}

}; // namespace smacc