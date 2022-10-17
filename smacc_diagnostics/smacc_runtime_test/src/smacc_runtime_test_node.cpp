#include <ros/ros.h>
#include <rosgraph_msgs/Log.h>
#include <smacc_msgs/SmaccTransitionLogEntry.h>
#include <boost/signals2.hpp>
#include <memory>
#include <vector>

class SmaccTestRuntimeNode;
class TimeoutFailureTestPolicy;
class ReachedStateSuccessTestPolicy;

//---------------------------------------------------------------------------------------------------
class TestPolicy
{
public:
  SmaccTestRuntimeNode* owner_;

  virtual void init(XmlRpc::XmlRpcValue& initXmlRpcValue)
  {
  }

  virtual void update()
  {
  }
};
//---------------------------------------------------------------------------------------------------

class SmaccTestRuntimeNode
{
public:
  ros::NodeHandle nh_;
  ros::Subscriber rosoutSub_;
  ros::Subscriber smaccTransitionSub_;


  std::vector<std::shared_ptr<TestPolicy>> testPolicies;
  boost::signals2::signal<void(const rosgraph_msgs::Log&)> onRosOutMsg;
  boost::signals2::signal<void(const smacc_msgs::SmaccTransitionLogEntry&)> onSmaccTransition;

  template <typename T>
  std::shared_ptr<TestPolicy> tryTestPolicyFactory(std::string matchname, XmlRpc::XmlRpcValue& initXmlRpcValue)
  {
    std::shared_ptr<TestPolicy> policy = nullptr;
    ROS_INFO_STREAM(initXmlRpcValue);

    auto type = (std::string)initXmlRpcValue["type"];

    if (type == matchname)
    {
      auto specificPolicy = std::make_shared<T>();
      policy = std::dynamic_pointer_cast<TestPolicy>(specificPolicy);
      testPolicies.push_back(policy);

      policy->owner_ = this;
      policy->init(initXmlRpcValue);
    }

    return policy;
  }

  void init()
  {
    ROS_INFO("[SmaccTestruntimeNode] subscribing rosout");

    ros::NodeHandle private_nh("~");
    std::string stateMachineROSParamWs;
    if (private_nh.getParam("state_machine_rosparam_ws", stateMachineROSParamWs))
    {
      ROS_INFO_STREAM("state machine param ws: " << stateMachineROSParamWs);
    }

    rosoutSub_ = nh_.subscribe("/rosout", 1, &SmaccTestRuntimeNode::onRosOutCallback, this);
    smaccTransitionSub_ = nh_.subscribe(stateMachineROSParamWs + "/smacc/transition_log", 1, &SmaccTestRuntimeNode::onSmaccTransitionCallback, this);

    XmlRpc::XmlRpcValue successSwitchParam;
    private_nh.getParam("success_switch", successSwitchParam);
    ROS_INFO_STREAM("[SmaccTestruntimeNode] success switch" << successSwitchParam);
    ROS_ASSERT(successSwitchParam.getType() == XmlRpc::XmlRpcValue::TypeArray);

    for (int32_t i = 0; i < successSwitchParam.size(); ++i)
    {
      auto initXmlRpcValue = successSwitchParam[i];
      if (tryTestPolicyFactory<ReachedStateSuccessTestPolicy>("state_reached", initXmlRpcValue) != nullptr)
        continue;
    }

    XmlRpc::XmlRpcValue failureSwitchParam;
    private_nh.getParam("failure_switch", failureSwitchParam);
    ROS_INFO_STREAM("[SmaccTestruntimeNode] failure switch" << failureSwitchParam);
    ROS_ASSERT(failureSwitchParam.getType() == XmlRpc::XmlRpcValue::TypeArray);

    for (int32_t i = 0; i < failureSwitchParam.size(); ++i)
    {
      auto initXmlRpcValue = failureSwitchParam[i];
      if (tryTestPolicyFactory<TimeoutFailureTestPolicy>("timeout", initXmlRpcValue) != nullptr)
        continue;
    }
  }
  void success(std::string msg)
  {
    ROS_INFO_STREAM("SmaccRuntimeTestNode policy thrown success exit: " << msg);
    exit(-1);
  }

  void failure(std::string msg)
  {
    ROS_FATAL_STREAM("SmaccRuntimeTestNode policy thrown failure exit: " << msg);
    exit(-1);
  }

  void update()
  {
    for (auto& policy : testPolicies)
    {
      policy->update();
    }
  }

  void onRosOutCallback(const rosgraph_msgs::Log& msg)
  {
      this->onRosOutMsg(msg);
  }

  void onSmaccTransitionCallback(const smacc_msgs::SmaccTransitionLogEntry& msg)
  {
      this->onSmaccTransition(msg);
  }
};

//---------------------------------------------------------------------------------------------------
class TimeoutFailureTestPolicy : public TestPolicy
{
  ros::Duration timeout_;
  ros::Time startTime_;

  virtual void init(XmlRpc::XmlRpcValue& initXmlRpcValue) override
  {
    ROS_INFO("[TimeoutFailureTestPolicy] initializating");
    startTime_ = ros::Time::now();
    timeout_ = ros::Duration((double)initXmlRpcValue["duration"]);

    ROS_INFO_STREAM("[TimeoutFailureTestPolicy] duration: " << timeout_);
  }

  virtual void update()
  {
    auto now = ros::Time::now();

    auto elapsed = now - startTime_;

    if (elapsed > timeout_)
    {
      this->owner_->failure(std::string("timeout failure: ") + std::to_string(timeout_.toSec()));
    }
  }
};
//---------------------------------------------------------------------------------------------------
class ReachedStateSuccessTestPolicy : public TestPolicy
{
  std::string targetStateName_;
  virtual void init(XmlRpc::XmlRpcValue& initXmlRpcValue) override
  {
    ROS_INFO("[ReachedStateSuccessTestPolicy] initializating");
    targetStateName_ = ((std::string)initXmlRpcValue["state_name"]);
    ROS_INFO_STREAM("[ReachedStateSuccessTestPolicy] success state: " << targetStateName_);
    this->owner_->onSmaccTransition.connect(
    [=](auto& msg)
    {
        ROS_INFO_STREAM("[ReachedStateSuccessTestPolicy] received state: " << msg.transition.destiny_state_name);

        if(msg.transition.destiny_state_name == targetStateName_)
        {
            this->owner_->success(std::string("success destiny state ") + targetStateName_);
        }
    });
  }

  virtual void update()
  {

  }
};
//---------------------------------------------------------------------------------------------------

int main(int argc, char** argv)
{
  ros::init(argc, argv, "smacc_runtime_test_node");

  SmaccTestRuntimeNode testNode;
  testNode.init();

  ros::Rate r(20);

  while (ros::ok())
  {
    testNode.update();
    ros::spinOnce();
    r.sleep();
  }
  // ros::Subscriber sub = nh.subscribe();
}
