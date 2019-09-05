#include <smacc_core/orthogonal.h>

namespace smacc
{
    void Orthogonal::setStateMachine(ISmaccStateMachine* value)
    {
        this->stateMachine_ = value;
    }

    void Orthogonal::setStateBehavior(smacc::SmaccStateBehavior* statebehavior)
    {
       if(statebehavior !=nullptr)
      {
        ROS_INFO("Behavioral State by orthogonal");
        statebehavior->stateMachine = this->stateMachine_;
        currentBehavior = statebehavior;
      }
      else
      {
        ROS_INFO("Not behavioral State by orthogonal");
      }        
    }

    void Orthogonal::onEntry()
    {
        ROS_INFO("Orthogonal OnEntry, current Behavior: %ld", long(currentBehavior));
        if(currentBehavior!= nullptr)
        {
            currentBehavior->onEntry();
        }
    }

    void Orthogonal::onExit()
    {
        ROS_INFO("Orthogonal OnExit, current Behavior: %ld", long(currentBehavior));
        if(currentBehavior!=nullptr)
        {
            currentBehavior->onExit();
            #warning improve this moving to shared pointers
            currentBehavior = nullptr;
        }
    }
}