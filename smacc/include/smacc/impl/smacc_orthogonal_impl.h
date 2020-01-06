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
        const std::type_info *componentkey = &typeid(SmaccComponentType);
        std::shared_ptr<SmaccComponentType> ret;

        auto it = this->components_.find(componentkey);

        if (it == this->components_.end())
        {
            auto tname = demangledTypeName<SmaccComponentType>();
            ROS_DEBUG("%s smacc component is required. Creating a new instance.", tname.c_str());

            ret = std::shared_ptr<SmaccComponentType>(new SmaccComponentType(targs...));
            ret->initialize(this);
            ret->setStateMachine(this->stateMachine_);

            this->components_[componentkey] = ret; //std::dynamic_pointer_cast<smacc::ISmaccComponent>(ret);
            ROS_DEBUG("%s resource is required. Done.", tname.c_str());
        }
        else
        {
            ROS_DEBUG("%s resource is required. Found resource in cache.", demangledTypeName<SmaccComponentType>().c_str());
            ret = dynamic_pointer_cast<SmaccComponentType>(it->second);
        }

        ret->template configureEventSourceTypes<TOrthogonal, TClient>();

        return ret.get();
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
        client->setStateMachine(stateMachine_);

        client->template configureEventSourceTypes<TOrthogonal, TClient>();

        clients_.push_back(client);
        return client;
    }
};
}; // namespace smacc