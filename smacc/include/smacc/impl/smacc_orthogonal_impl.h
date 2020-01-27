/*****************************************************************************************************************
 * ReelRobotix Inc. - Software License Agreement      Copyright (c) 2018
 * 	 Authors: Pablo Inigo Blasco, Brett Aldrich
 *
 ******************************************************************************************************************/

#pragma once
#include <smacc/smacc_orthogonal.h>
#include <smacc/smacc_client.h>
#include <cassert>

namespace smacc
{

template <typename SmaccClientType>
void IOrthogonal::requiresClient(SmaccClientType *&storage)
{
    for (auto &client : clients_)
    {
        storage = dynamic_cast<SmaccClientType *>(client.get());
        if (storage != nullptr)
            return;
    }
}

template <typename SmaccComponentType>
void IOrthogonal::requiresComponent(SmaccComponentType *&storage)
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
template <typename TClientBehavior>
TClientBehavior *IOrthogonal::getClientBehavior()
{
    for (auto &cb : this->clientBehaviors_)
    {
        auto *ret = dynamic_cast<TClientBehavior *>(cb.get());
        if (ret != nullptr)
        {
            return ret;
        }
    }

    return nullptr;
}

inline const std::vector<std::shared_ptr<smacc::ISmaccClient>> &IOrthogonal::getClients()
{
    return clients_;
}

inline const std::vector<std::shared_ptr<smacc::SmaccClientBehavior>> &IOrthogonal::getClientBehaviors() const
{
    return this->clientBehaviors_;
}

template <typename T>
void IOrthogonal::setGlobalSMData(std::string name, T value)
{
    this->getStateMachine()->setGlobalSMData(name, value);
}

template <typename T>
bool IOrthogonal::getGlobalSMData(std::string name, T &ret)
{
    return this->getStateMachine()->getGlobalSMData(name, ret);
}

//inline
ISmaccStateMachine *IOrthogonal::getStateMachine() { return this->stateMachine_; }

template <typename TOrthogonal, typename TClient>
class ClientHandler : public TClient
{
public:
    template <typename... TArgs>
    ClientHandler(TArgs... args)
        : TClient(args...)
    {
    }

    template <typename SmaccComponentType, typename... TArgs>
    SmaccComponentType *createComponent(TArgs... targs)
    {
        return ISmaccClient::createComponent<SmaccComponentType, TOrthogonal, TClient,  TArgs...>(targs...);
    }

    virtual smacc::introspection::TypeInfo::Ptr getType() override
    {
        return smacc::introspection::TypeInfo::getTypeInfoFromType<TClient>();
    }
};

template <typename TOrthogonal>
class Orthogonal : public IOrthogonal
{
public:
    template <typename TClient, typename... TArgs>
    std::shared_ptr<ClientHandler<TOrthogonal, TClient>> createClient(TArgs... args)
    {
        //static_assert(std::is_base_of<IOrthogonal, TObjectTag>::value, "The object Tag must be the orthogonal type where the client was created");
        // if (typeid(*this) != typeid(TObjectTag))
        // {
        //     ROS_ERROR_STREAM("Error creating client. The object Tag must be the type of the orthogonal where the client was created:" << demangleType(typeid(*this)) << ". The object creation was skipped and a nullptr was returned");
        //     return nullptr;
        // }

        ROS_INFO("[%s] creates a client of type '%s' and object tag '%s'",
                 demangleType(typeid(*this)).c_str(),
                 demangledTypeName<TClient>().c_str(),
                 demangledTypeName<TOrthogonal>().c_str());

        auto client = std::make_shared<ClientHandler<TOrthogonal, TClient>>(args...);
        client->setStateMachine(getStateMachine());

        client->template configureEventSourceTypes<TOrthogonal, TClient>();

        // it is stored the client (not the client handler)
        clients_.push_back(client);
        return client;
    }
};
}; // namespace smacc