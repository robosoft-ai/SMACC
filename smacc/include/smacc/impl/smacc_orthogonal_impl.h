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
bool ISmaccOrthogonal::requiresClient(SmaccClientType *&storage)
{
    for (auto &client : clients_)
    {
        storage = dynamic_cast<SmaccClientType *>(client.get());
        if (storage != nullptr)
            return true;
    }

    auto requiredClientName = demangledTypeName<SmaccClientType>();
    ROS_WARN_STREAM("Required client ["<< requiredClientName<< "] not found in current orthogonal. Searching in other orthogonals.");

    for (auto &orthoentry : this->getStateMachine()->getOrthogonals())
    {
        for (auto &client : orthoentry.second->getClients())
        {
            storage = dynamic_cast<SmaccClientType *>(client.get());
            if (storage != nullptr)
            {
                ROS_WARN_STREAM("Required client  ["<< requiredClientName<<"] found in other orthogonal.");
                return true;
            }
        }
    }

    ROS_ERROR_STREAM("Required client ["<< requiredClientName<< "] not found even in other orthogonals. Returning null pointer. If the requested client is used may result in a segmentation fault.");
    return false;
}

template <typename SmaccClientType>
bool ISmaccOrthogonal::requiresClient(std::shared_ptr<SmaccClientType> &storage)
{
    for (auto &client : clients_)
    {
        storage = std::dynamic_pointer_cast<SmaccClientType>(client);
        if (storage != nullptr)
            return true;
    }

    auto requiredClientName = demangledTypeName<SmaccClientType>();
    ROS_WARN_STREAM("Required client ["<< requiredClientName<< "] not found in current orthogonal. Searching in other orthogonals.");

    for (auto &orthoentry : this->getStateMachine()->getOrthogonals())
    {
        for (auto &client : orthoentry.second->getClients())
        {
            storage = std::dynamic_pointer_cast<SmaccClientType>(client);
            if (storage != nullptr)
            {
                ROS_WARN_STREAM("Required client  ["<< requiredClientName<<"] found in other orthogonal.");
                return true;
            }
        }
    }

    ROS_ERROR_STREAM("Required client ["<< requiredClientName<< "] not found even in other orthogonals. Returning null pointer. If the requested client is used may result in a segmentation fault.");
    return false;
}

template <typename SmaccComponentType>
void ISmaccOrthogonal::requiresComponent(SmaccComponentType *&storage)
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

template <typename SmaccComponentType>
void ISmaccOrthogonal::requiresComponent(std::shared_ptr<SmaccComponentType> &storage)
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

template <typename TOrthogonal, typename TClient>
void ISmaccOrthogonal::assignClientToOrthogonal(TClient* client)
{
    client->setStateMachine(getStateMachine());
    client->setOrthogonal(this);

    client->template onOrthogonalAllocation<TOrthogonal, TClient>();
}

template <typename TClientBehavior>
TClientBehavior *ISmaccOrthogonal::getClientBehavior()
{
    for (auto &cb : this->clientBehaviors_.back())
    {
        auto *ret = dynamic_cast<TClientBehavior *>(cb.get());
        if (ret != nullptr)
        {
            return ret;
        }
    }

    return nullptr;
}

inline const std::vector<std::shared_ptr<smacc::ISmaccClient>> &ISmaccOrthogonal::getClients()
{
    return clients_;
}

inline const std::vector<std::vector<std::shared_ptr<smacc::ISmaccClientBehavior>>> &ISmaccOrthogonal::getClientBehaviors() const
{
    return this->clientBehaviors_;
}

template <typename T>
void ISmaccOrthogonal::setGlobalSMData(std::string name, T value)
{
    this->getStateMachine()->setGlobalSMData(name, value);
}

template <typename T>
bool ISmaccOrthogonal::getGlobalSMData(std::string name, T &ret)
{
    return this->getStateMachine()->getGlobalSMData(name, ret);
}

//inline
ISmaccStateMachine *ISmaccOrthogonal::getStateMachine() { return this->stateMachine_; }

template <typename TOrthogonal, typename TClient>
class ClientHandler : public TClient
{
public:
    template <typename... TArgs>
    ClientHandler(TArgs... args)
        : TClient(args...)
    {
    }

    ClientHandler()
        : TClient()
    {
    }

    template <typename SmaccComponentType, typename... TArgs>
    SmaccComponentType *createComponent(TArgs... targs)
    {
        return ISmaccClient::createComponent<SmaccComponentType, TOrthogonal, TClient, TArgs...>(targs...);
    }

    template <typename SmaccComponentType, typename... TArgs>
    SmaccComponentType *createNamedComponent(std::string name, TArgs... targs)
    {
        return ISmaccClient::createNamedComponent<SmaccComponentType, TOrthogonal, TClient, TArgs...>(name, targs...);
    }

    virtual smacc::introspection::TypeInfo::Ptr getType() override
    {
        return smacc::introspection::TypeInfo::getTypeInfoFromType<TClient>();
    }
};

template <typename TOrthogonal>
class Orthogonal : public ISmaccOrthogonal
{
public:
    template <typename TClient, typename... TArgs>
    std::shared_ptr<ClientHandler<TOrthogonal, TClient>> createClient(TArgs... args)
    {
        //static_assert(std::is_base_of<ISmaccOrthogonal, TOrthogonal>::value, "The object Tag must be the orthogonal type where the client was created");
        // if (typeid(*this) != typeid(TOrthogonal))
        // {
        //     ROS_ERROR_STREAM("Error creating client. The object Tag must be the type of the orthogonal where the client was created:" << demangleType(typeid(*this)) << ". The object creation was skipped and a nullptr was returned");
        //     return nullptr;
        // }

        ROS_INFO("[%s] creates a client of type '%s' and object tag '%s'",
                    demangleType(typeid(*this)).c_str(),
                    demangledTypeName<TClient>().c_str(),
                    demangledTypeName<TOrthogonal>().c_str()
                );

        auto client = std::make_shared<ClientHandler<TOrthogonal, TClient>>(args...);
        this->template assignClientToOrthogonal<TOrthogonal, TClient>(client.get());

        // it is stored the client (not the client handler)
        clients_.push_back(client);

        return client;
    }
};
}; // namespace smacc
