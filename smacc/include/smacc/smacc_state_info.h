#pragma once
#include <memory>
#include <map>
#include <functional>
#include <vector>
#include <list>

namespace smacc
{
class ISmaccState;
class SmaccStateMachineInfo;
class SmaccStateInfo;

struct StateBehaviorInfoEntry
{
    std::function<void(smacc::ISmaccState *)> factoryFunction;
    const std::type_info *behaviorType;
    const std::type_info *orthogonalType;
};

struct SmaccTransitionInfo
{
    int index;
    std::shared_ptr<SmaccStateInfo> sourceState;
    std::shared_ptr<SmaccStateInfo> destinyState;
    const std::type_info* eventType;
};

class SmaccStateInfo : public std::enable_shared_from_this<SmaccStateInfo>
{
public:
    static std::map<const std::type_info *, std::shared_ptr<std::vector<StateBehaviorInfoEntry>>> staticBehaviorInfo;

    bool active_;
    std::string fullStateName;
    std::string demangledStateName;

    std::shared_ptr<SmaccStateMachineInfo> stateMachine_;
    std::shared_ptr<SmaccStateInfo> parentState_;
    std::vector<SmaccTransitionInfo> transitions_;

    std::vector<std::shared_ptr<SmaccStateInfo>> children_;
    int depth_;

    SmaccStateInfo(std::shared_ptr<SmaccStateInfo> parentState, std::shared_ptr<SmaccStateMachineInfo> stateMachineInfo);

    inline int depth() const { return depth_; }

    void getAncestors(std::list<std::shared_ptr<SmaccStateInfo>> &ancestorsList);

    std::string getFullPath();

    template <typename StateType>
    std::shared_ptr<SmaccStateInfo> createChildState();

    template <typename EvType>
    void declareTransition(std::shared_ptr<SmaccStateInfo> &dstState);

    template<typename EvSource, template<typename> typename EvType >
    void declareTransition(std::shared_ptr<SmaccStateInfo> &dstState);

    const std::string &toShortName() const;
};
} // namespace smacc