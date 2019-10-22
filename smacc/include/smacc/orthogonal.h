#pragma once
#include <smacc/common.h>

namespace smacc
{

class Orthogonal
{
public:
    void setStateMachine(ISmaccStateMachine *value);

    void setStateBehavior(std::shared_ptr<smacc::SmaccSubStateBehavior> statebehavior);

    void onEntry();

    void onExit();

    virtual std::string getName() const;

    template <typename T>
    T *createClient();

    template <typename SmaccComponentType>
    void requiresComponent(SmaccComponentType *&storage, bool verbose = false);

    template <typename SmaccClientType>
    void requiresClient(SmaccClientType *&storage, bool verbose = false);

    const std::vector<smacc::ISmaccClient *> &getClients()
    {
        return clients_;
    }

private:
    virtual void onInitialize();

    ISmaccStateMachine *stateMachine_;

    std::shared_ptr<smacc::SmaccSubStateBehavior> currentBehavior_;

    std::vector<smacc::ISmaccClient *> clients_;
};

} // namespace smacc