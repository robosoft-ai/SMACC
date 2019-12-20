#pragma once
#include <smacc/common.h>
#include <utility>

namespace smacc
{

class Orthogonal
{
public:
    void setStateMachine(ISmaccStateMachine *value);

    void setStateBehavior(std::shared_ptr<smacc::SmaccClientBehavior> statebehavior);

    void onEntry();

    void onExit();

    virtual std::string getName() const;

    template <typename TObjectTag, typename TClient, typename ...TArgs>
    std::shared_ptr<TClient> createClient(TArgs... args);

    template <typename SmaccComponentType>
    void requiresComponent(SmaccComponentType *&storage, bool verbose = false);

    template <typename SmaccClientType>
    void requiresClient(SmaccClientType *&storage, bool verbose = false);

    inline const std::vector<std::shared_ptr<smacc::ISmaccClient>> &getClients()
    {
        return clients_;
    }

    inline smacc::SmaccClientBehavior* getCurrentBehavior()
    {
        return this->currentBehavior_.get();
    }

private:
    virtual void onInitialize();

    ISmaccStateMachine *stateMachine_;

    std::shared_ptr<smacc::SmaccClientBehavior> currentBehavior_;

    std::vector<std::shared_ptr<smacc::ISmaccClient>> clients_;
};

} // namespace smacc