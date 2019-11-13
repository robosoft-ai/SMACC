/*****************************************************************************************************************
 * ReelRobotix Inc. - Software License Agreement      Copyright (c) 2018
 * 	 Authors: Pablo Inigo Blasco, Brett Aldrich
 *
 ******************************************************************************************************************/
#pragma once

#include <smacc/interface_components/smacc_action_client.h>
#include <smacc/impl/smacc_component_impl.h>
#include <boost/optional/optional_io.hpp>

#include <queue>

namespace smacc
{
// Smacc Action Clients (AKA resources or plugins) can inherit from this object
// inhteriting from this class works as a template .h library. That is why the code
// implementation is located here.
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

    SmaccActionClientBase(int feedback_queue_size = 10)
        : ISmaccActionClient()
    {
        feedback_queue_size_ = feedback_queue_size;
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
            boost::split(strs,*name_,boost::is_any_of("::"));
            std::string classname = strs.back();
            name_ = classname;
        }

        client_ = std::make_shared<ActionClient>(*name_);
    }

    virtual ~SmaccActionClientBase()
    {
    }

    virtual void cancelGoal()
    {
        ROS_INFO("Cancelling goal of %s", this->getName().c_str());
        client_->cancelGoal();
    }

    virtual SimpleClientGoalState getState() override
    {
        return client_->getState();
    }

    virtual bool hasFeedback() override
    {
        return !feedback_queue_.empty();
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

        stateMachine_->registerActionClientRequest(this);
    }

protected:
    std::shared_ptr<ActionClient> client_;

    int feedback_queue_size_;
    std::list<Feedback> feedback_queue_;

    int result_queue_size_;
    std::list<Result> result_queue_;

    void onFeedback(const FeedbackConstPtr &feedback)
    {
        Feedback copy = *feedback;
        feedback_queue_.push_back(copy);
        ROS_DEBUG("FEEDBACK MESSAGE RECEIVED, enqueuing Queue Size: %ld", feedback_queue_.size());
        if (feedback_queue_.size() > feedback_queue_size_)
        {
            feedback_queue_.pop_front();
        }
    }

    void onResult(const SimpleClientGoalState &state, const ResultConstPtr &result)
    {
        Result copy = *result;
        result_queue_.push_back(copy);
        ROS_DEBUG("RESULT MESSAGE RECEIVED, enqueuing Queue Size: %ld", result_queue_.size());
        if (result_queue_.size() > result_queue_size_)
        {
            result_queue_.pop_front();
        }
    }

    virtual void postEvent(SmaccScheduler *scheduler, SmaccScheduler::processor_handle processorHandle) override
    {
        Result result_msg;
        auto *actionResultEvent = new EvActionResult<TDerived>();
        actionResultEvent->client = this;

        ROS_INFO("[%s] Action client received a result of the request, Queue Size: %ld", this->getName().c_str(), result_queue_.size());

        if (!result_queue_.empty())
        {
            ROS_INFO("[%s]Popping RESULT MESSAGE, Queue Size: %ld", this->getName().c_str(), result_queue_.size());
            result_msg = result_queue_.front();
            result_queue_.pop_front();
            ROS_INFO("[%s]popped RESULT MESSAGE, Queue Size: %ld", this->getName().c_str(), result_queue_.size());
            actionResultEvent->resultMessage = result_msg;

            //scheduler->queue_event(processorHandle, actionResultEvent);

            const auto &resultType = actionResultEvent->getResultState();
            ROS_INFO("[%s] request result: %s", this->getName().c_str(), resultType.toString().c_str());

            {
                if (resultType == actionlib::SimpleClientGoalState::SUCCEEDED)
                {
                    ROS_INFO("[%s] request result: Success", this->getName().c_str());
                    auto *successEvent = new EvActionSucceded<TDerived>();

                    successEvent->client = this;
                    successEvent->resultMessage = result_msg;

                    ROS_INFO("Posting EVENT %s", demangleSymbol(typeid(successEvent).name()).c_str());
                    stateMachine_->postEvent(successEvent);
                    //scheduler->queue_event(processorHandle, successEvent);
                }
                else if (resultType == actionlib::SimpleClientGoalState::ABORTED)
                {
                    ROS_INFO("[%s] request result: Aborted", this->getName().c_str());
                    auto *abortedEvent = new EvActionAborted<TDerived>();

                    abortedEvent->client = this;
                    abortedEvent->resultMessage = result_msg;

                    ROS_INFO("[%s] Posting EVENT %s", this->getName().c_str(), demangleSymbol(typeid(abortedEvent).name()).c_str());
                    stateMachine_->postEvent(abortedEvent);
                    //scheduler->queue_event(processorHandle, abortedEvent);
                }
                else if (resultType == actionlib::SimpleClientGoalState::REJECTED)
                {
                    ROS_INFO("[%s] request result: Rejected", this->getName().c_str());
                    auto *rejectedEvent = new EvActionRejected<TDerived>();

                    rejectedEvent->client = this;
                    rejectedEvent->resultMessage = result_msg;

                    ROS_INFO("Posting EVENT %s", demangleSymbol(typeid(rejectedEvent).name()).c_str());
                    //scheduler->queue_event(processorHandle, rejectedEvent);
                    stateMachine_->postEvent(rejectedEvent);
                }
                else if (resultType == actionlib::SimpleClientGoalState::PREEMPTED)
                {
                    ROS_INFO("[%s] request result: Preempted", this->getName().c_str());
                    auto *preemtedEvent = new EvActionPreempted<TDerived>();

                    preemtedEvent->client = this;
                    preemtedEvent->resultMessage = result_msg;

                    ROS_INFO("Posting EVENT %s", demangleSymbol(typeid(preemtedEvent).name()).c_str());
                    //scheduler->queue_event(processorHandle, preemtedEvent);
                    stateMachine_->postEvent(preemtedEvent);
                }
                else
                {
                    ROS_INFO("[%s] request result: NOT HANDLED TYPE: %s", this->getName().c_str(), resultType.toString().c_str());
                }
            }
        }
        else
        {
            ROS_WARN("action client result queue is empty");
        }
    }

    virtual void postFeedbackEvent(SmaccScheduler *scheduler, SmaccScheduler::processor_handle processorHandle) override
    {
        boost::intrusive_ptr<EvActionFeedback<Feedback>> actionFeedbackEvent = new EvActionFeedback<Feedback>();
        actionFeedbackEvent->client = this;

        bool ok = false;
        if (!feedback_queue_.empty())
        {
            ROS_DEBUG("[%s]Popping FEEDBACK MESSAGE, Queue Size: %ld", this->getName().c_str(), feedback_queue_.size());
            Feedback feedback_msg = feedback_queue_.front();
            feedback_queue_.pop_front();
            ROS_DEBUG("[%s]popped FEEDBACK MESSAGE, Queue Size: %ld", this->getName().c_str(), feedback_queue_.size());
            actionFeedbackEvent->feedbackMessage = feedback_msg;
            ok = true;
        }
        else
        {
            ok = false;
        };

        if (ok)
        {
            //ROS_INFO("Sending feedback event");
            scheduler->queue_event(processorHandle, actionFeedbackEvent);
        }
    }

    friend class SignalDetector;
};
} // namespace smacc