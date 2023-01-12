#pragma once

#include <smacc/client_bases/smacc_publisher_client.h>
#include <smacc/smacc.h>
#include <std_msgs/UInt16.h>

namespace sm_atomic_threadable {
template <int TIndex>
class ClTalker : public smacc::client_bases::SmaccPublisherClient {
 public:
  ClTalker(const std::string& topic) {
    this->configure<std_msgs::UInt16>(topic);
  }
};
}  // namespace sm_atomic_threadable
