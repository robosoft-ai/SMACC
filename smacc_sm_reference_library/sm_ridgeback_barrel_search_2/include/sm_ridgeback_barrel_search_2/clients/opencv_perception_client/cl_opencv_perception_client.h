#pragma once

#include <smacc/client_bases/smacc_subscriber_client.h>
#include <std_msgs/Int32.h>

namespace sm_ridgeback_barrel_search_2
{
namespace cl_opencv_perception
{
class ClOpenCVPerception : public smacc::client_bases::SmaccSubscriberClient<std_msgs::Int32>
{
public:

  ClOpenCVPerception(std::string topicname = "/detected_color")
    : smacc::client_bases::SmaccSubscriberClient<std_msgs::Int32>(topicname)
  {
  }

  virtual ~ClOpenCVPerception()
  {
  }

  template <typename TOrthogonal, typename TSourceObject>
  void onOrthogonalAllocation()
  {
    smacc::client_bases::SmaccSubscriberClient<std_msgs::Int32>::onOrthogonalAllocation<TOrthogonal, TSourceObject>();
  }
};
}
}
