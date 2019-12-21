#pragma once
#include <smacc/common.h>
#include <utility>

namespace smacc
{

class Orthogonal
{
public:
    void setStateMachine(ISmaccStateMachine *value);

    void addClientBehavior(std::shared_ptr<smacc::SmaccClientBehavior> statebehavior);

    void onEntry();

    void onExit();

    virtual std::string getName() const;

    template <typename TObjectTag, typename TClient, typename... TArgs>
    std::shared_ptr<TClient> createClient(TArgs... args);

    template <typename SmaccComponentType>
    void requiresComponent(SmaccComponentType *&storage);

    template <typename SmaccClientType>
    void requiresClient(SmaccClientType *&storage);

    inline const std::vector<std::shared_ptr<smacc::ISmaccClient>> &getClients()
    {
        return clients_;
    }

    inline const std::vector<std::shared_ptr<smacc::SmaccClientBehavior>> &getClientBehaviors() const
    {
        return this->clientBehaviors_;
    }

private:
    virtual void onInitialize();

    ISmaccStateMachine *stateMachine_;

    std::vector<std::shared_ptr<smacc::SmaccClientBehavior>> clientBehaviors_;

    std::vector<std::shared_ptr<smacc::ISmaccClient>> clients_;
};

} // namespace smacc