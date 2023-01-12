#pragma once

#include <smacc/smacc.h>
#include <std_msgs/UInt16.h>

namespace sm_atomic_threadable {
template <int TIndex>
class ClListener
    : public smacc::client_bases::SmaccSubscriberClient<std_msgs::UInt16> {
 public:
  ClListener(const std::string& topic)
      : smacc::client_bases::SmaccSubscriberClient<std_msgs::UInt16>{topic} {}
};
}  // namespace sm_atomic_threadable
