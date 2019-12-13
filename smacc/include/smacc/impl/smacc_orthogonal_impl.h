#pragma once
#include <smacc/smacc_orthogonal.h>
#include <smacc/smacc_client.h>

namespace smacc
{
template <typename TObjectTag, typename TClient, typename ...TArgs>
std::shared_ptr<TClient> Orthogonal::createClient(TArgs... args)
{
    auto client = std::make_shared<TClient>(args...);
    client->setStateMachine(stateMachine_);

    client->template setObjectTagIdentifier<TClient, TObjectTag>();
    clients_.push_back(client);
    return client;
}

template <typename SmaccComponentType>
void Orthogonal::requiresComponent(SmaccComponentType *&storage, bool verbose)
{
    if (stateMachine_ == nullptr)
    {
        ROS_ERROR("Cannot use the requiresComponent funcionality from an orthogonal before onInitialize");
    }
    else
    {
        stateMachine_->requiresComponent(storage, verbose);
    }
}

template <typename SmaccClientType>
void Orthogonal::requiresClient(SmaccClientType *&storage, bool verbose)
{
    for (auto& client : clients_)
    {
        storage = dynamic_cast<SmaccClientType *>(client.get());
        if (storage != nullptr)
            break;
    }
}

}; // namespace smacc