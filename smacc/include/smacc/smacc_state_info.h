#pragma once
#include <memory>
#include <map>

namespace smacc
{

class SmaccStateInfo: public std::enable_shared_from_this<SmaccStateInfo>
{
public:
    bool active_;
    std::string fullStateName;
    std::string demangledStateName;

    std::shared_ptr<SmaccStateMachineInfo> stateMachine_;
    std::shared_ptr<SmaccStateInfo> parentState_;
    std::map<std::string,std::shared_ptr<SmaccStateInfo>> transitions_;

    std::vector<std::shared_ptr<SmaccStateInfo>> children_;
    int depth_;

    SmaccStateInfo(std::shared_ptr<SmaccStateInfo> parentState, std::shared_ptr<SmaccStateMachineInfo> stateMachineInfo)
    {
        parentState_ = parentState;
        stateMachine_ = stateMachineInfo;

        if(parentState_!=nullptr)
            depth_ = parentState->depth_ +1;
    }

    inline int depth() const{return depth_;}


    std::string getFullPath()
    {
        if( parentState_==nullptr)
            return this->toShortName();
        else
            return this->parentState_->getFullPath() + "/" + this->toShortName();
    }

    template <typename StateType>
    std::shared_ptr<SmaccStateInfo> createChildState();

    template <typename EvType>
    void declareTransition(std::shared_ptr<SmaccStateInfo>& dstState);

    const std::string& toShortName() const
    {
        return this->demangledStateName;
    }
};

}