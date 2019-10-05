#pragma once
#include <smacc/common.h>
#include <smacc/smacc_state_machine.h>

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
    T *createClient()
    {
        auto *client = new T();
        client->setStateMachine(stateMachine_);
        
        clients_.push_back(client);
        return client;
    }

    template <typename SmaccComponentType>
    void requiresComponent(SmaccComponentType *&storage, bool verbose = false)
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
    void requiresClient(SmaccClientType *&storage, bool verbose = false)
    {
        for (auto *client : clients_)
        {
            storage = dynamic_cast<SmaccClientType *>(client);
            if (storage != nullptr)
                break;
        }
    }

private:

    virtual void onInitialize();

    ISmaccStateMachine *stateMachine_;

    std::shared_ptr<smacc::SmaccSubStateBehavior> currentBehavior_;

    std::vector<smacc::ISmaccClient *> clients_;
};

} // namespace smacc