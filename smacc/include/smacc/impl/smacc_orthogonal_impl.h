#pragma once
#include <smacc/orthogonal.h>

namespace smacc
{
template <typename T>
T *Orthogonal::createClient()
{
    auto *client = new T();
    client->setStateMachine(stateMachine_);

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
    for (auto *client : clients_)
    {
        storage = dynamic_cast<SmaccClientType *>(client);
        if (storage != nullptr)
            break;
    }
}

}; // namespace smacc