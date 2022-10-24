/*****************************************************************************************************************
 * ReelRobotix Inc. - Software License Agreement      Copyright (c) 2018
 * 	 Authors: Pablo Inigo Blasco, Brett Aldrich
 *
 ******************************************************************************************************************/

#pragma once

#include <smacc/smacc_client.h>
#include <boost/optional/optional_io.hpp>

namespace smacc
{
namespace client_bases
{
class SmaccPublisherClient : public smacc::ISmaccClient
{
public:
  boost::optional<std::string> topicName;
  boost::optional<int> queueSize;

  SmaccPublisherClient()
  {
    initialized_ = false;
  }

  virtual ~SmaccPublisherClient()
  {
    pub_.shutdown();
  }

  template <typename MessageType>
  void configure(std::string topicName)
  {
    this->topicName = topicName;
    if (!initialized_)
    {
      if (!this->topicName)
      {
        ROS_ERROR("topic publisher with no topic name set. Skipping advertising.");
        return;
      }
      if (!queueSize)
        queueSize = 1;

      ROS_INFO_STREAM("[" << this->getName() << "] Client Publisher to topic: " << topicName);
      pub_ = nh_.advertise<MessageType>(*(this->topicName), *queueSize);

      this->initialized_ = true;
    }
  }

  template <typename MessageType>
  void publish(const MessageType &msg)
  {
    pub_.publish(msg);
  }

protected:
  ros::NodeHandle nh_;
  ros::Publisher pub_;

private:
  bool initialized_;
};
} // namespace client_bases
} // namespace smacc
