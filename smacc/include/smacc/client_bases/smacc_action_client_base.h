#pragma once

#include <smacc/client_bases/smacc_action_client.h>
#include <boost/signals2.hpp>
#include <boost/optional/optional_io.hpp>

namespace smacc
{

template <typename TDerived, typename ActionType>
class SmaccActionClientBase : public ISmaccActionClient
{
public:
    // Inside this macro you can find the typedefs for Goal and other types
    ACTION_DEFINITION(ActionType);
    typedef actionlib::SimpleActionClient<ActionType> ActionClient;
    typedef actionlib::SimpleActionClient<ActionType> GoalHandle;

    typedef typename ActionClient::SimpleDoneCallback SimpleDoneCallback;
    typedef typename ActionClient::SimpleActiveCallback SimpleActiveCallback;
    typedef typename ActionClient::SimpleFeedbackCallback SimpleFeedbackCallback;

    SmaccActionClientBase()
        : ISmaccActionClient()
    {
    }

    static std::string getEventLabel()
    {
        auto type = TypeInfo::getTypeInfoFromType<ActionType>();
        return type->getNonTemplatetypename();
    }

    boost::optional<std::string> name_;

    virtual void initialize() override
    {
        if (!name_)
        {
            name_ = demangleSymbol(typeid(*this).name());
            std::vector<std::string> strs;
            boost::split(strs, *name_, boost::is_any_of("::"));
            std::string classname = strs.back();
            name_ = classname;
        }

        client_ = std::make_shared<ActionClient>(*name_);
    }

    virtual ~SmaccActionClientBase()
    {
    }

    virtual void cancelGoal() override
    {
        ROS_INFO("Cancelling goal of %s", this->getName().c_str());
        client_->cancelGoal();
    }

    virtual SimpleClientGoalState getState() override
    {
        return client_->getState();
    }

    void sendGoal(Goal &goal)
    {
        ROS_INFO_STREAM("Sending goal to actionserver located in " << *(this->name_) << "\"");

        if (!client_->isServerConnected())
        {
            ROS_INFO("%s [at %s]: not connected with actionserver, waiting ...", getName().c_str(), getNamespace().c_str());
            client_->waitForServer();
        }

        ROS_INFO_STREAM(getName() << ": Goal Value: " << std::endl
                                  << goal);

        SimpleDoneCallback done_cb = boost::bind(&SmaccActionClientBase<TDerived, ActionType>::onResult, this, _1, _2);
        SimpleActiveCallback active_cb;
        SimpleFeedbackCallback feedback_cb = boost::bind(&SmaccActionClientBase<TDerived, ActionType>::onFeedback, this, _1);

        client_->sendGoal(goal, done_cb, active_cb, feedback_cb);
    }

protected:
    std::shared_ptr<ActionClient> client_;

    void onFeedback(const FeedbackConstPtr &feedback_msg)
    {
        auto actionFeedbackEvent = new EvActionFeedback<Feedback>();
        actionFeedbackEvent->client = this;
        actionFeedbackEvent->feedbackMessage = *feedback_msg;
        this->postEvent(actionFeedbackEvent);
        ROS_WARN("FEEDBACK EVENT");
    }

    void onResult(const SimpleClientGoalState &state, const ResultConstPtr &result_msg)
    {
        auto *actionResultEvent = new EvActionResult<TDerived>();
        actionResultEvent->client = this;
        actionResultEvent->resultMessage = *result_msg;

        const auto &resultType = actionResultEvent->getResultState();
        ROS_INFO("[%s] request result: %s", this->getName().c_str(), resultType.toString().c_str());

        if (resultType == actionlib::SimpleClientGoalState::SUCCEEDED)
        {
            ROS_INFO("[%s] request result: Success", this->getName().c_str());
            auto *successEvent = new EvActionSucceeded<TDerived>();

            successEvent->client = this;
            successEvent->resultMessage = *result_msg;

            ROS_INFO("Posting EVENT %s", demangleSymbol(typeid(successEvent).name()).c_str());
            this->postEvent(successEvent);
        }
        else if (resultType == actionlib::SimpleClientGoalState::ABORTED)
        {
            ROS_INFO("[%s] request result: Aborted", this->getName().c_str());
            auto *abortedEvent = new EvActionAborted<TDerived>();

            abortedEvent->client = this;
            abortedEvent->resultMessage = *result_msg;

            ROS_INFO("[%s] Posting EVENT %s", this->getName().c_str(), demangleSymbol(typeid(abortedEvent).name()).c_str());
            this->postEvent(abortedEvent);
        }
        else if (resultType == actionlib::SimpleClientGoalState::REJECTED)
        {
            ROS_INFO("[%s] request result: Rejected", this->getName().c_str());
            auto *rejectedEvent = new EvActionRejected<TDerived>();

            rejectedEvent->client = this;
            rejectedEvent->resultMessage = *result_msg;

            ROS_INFO("Posting EVENT %s", demangleSymbol(typeid(rejectedEvent).name()).c_str());
            this->postEvent(rejectedEvent);
        }
        else if (resultType == actionlib::SimpleClientGoalState::PREEMPTED)
        {
            ROS_INFO("[%s] request result: Preempted", this->getName().c_str());
            auto *preemtedEvent = new EvActionPreempted<TDerived>();

            preemtedEvent->client = this;
            preemtedEvent->resultMessage = *result_msg;

            ROS_INFO("Posting EVENT %s", demangleSymbol(typeid(preemtedEvent).name()).c_str());
            this->postEvent(preemtedEvent);
        }
        else
        {
            ROS_INFO("[%s] request result: NOT HANDLED TYPE: %s", this->getName().c_str(), resultType.toString().c_str());
        }
    }
};
} // namespace smacc