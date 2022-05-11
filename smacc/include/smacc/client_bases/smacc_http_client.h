#pragma once

#include <cpr/cpr.h>  // Make sure this header is available in your include path
#include <smacc/impl/smacc_state_impl.h>
#include <smacc/smacc_client.h>

#include <boost/optional/optional_io.hpp>

namespace smacc {
namespace client_bases {

class SmaccHttpClient : public smacc::ISmaccClient {
 public:
  boost::optional<std::string> serverName;
  boost::optional<int> timeout;

  SmaccHttpClient() { initialized_ = false; }

  SmaccHttpClient(const std::string &serverName, const int &timeout) {
    this->serverName = serverName;
    this->timeout = timeout;
  }

  smacc::SmaccSignal<void(const cpr::Response &)> onResponseReceived_;

  template <typename T>
  boost::signals2::connection onResponseReceived(
      void (T::*callback)(const cpr::Response &), T *object) {
    return this->getStateMachine()->createSignalConnection(onResponseReceived_,
                                                           callback, object);
  }

  template <typename TOrthogonal, typename TSourceObject>
  void onOrthogonalAllocation() {}

  virtual void initialize() {
    if (!initialized_) {
      if (!timeout) timeout = 2000;  // 2s timeout default
      if (!serverName) {
        ROS_ERROR("Server URL not set, skipping initialisation");
      } else {
        ROS_INFO_STREAM("[" << this->getName()
                            << "] Initialising HTTP client for " << serverName);
        initialized_ = true;
      }
    }
  }

  // cpr::Header is an STL map-like object mapping string key-value pairs,
  // e.g. {"Content-Type", "application/json"}
  void makePostRequest(const std::string &path, const std::string &body,
                   const cpr::Header &header) {
    if (!initialized_) {
      ROS_ERROR("%s: not initialized.", this->getName().c_str());
      return;
    }
    if (!serverName) {
      ROS_ERROR("Server URL not set.");
      return;
    }
    request_ = cpr::PostCallback(
        [this](const cpr::Response &response) {
          onResponseReceived_(response);
        },
        cpr::Url{serverName.get() + path}, cpr::Body{{body}},
        cpr::Header{header}, cpr::Timeout{timeout.get()});
  }

  void makeGetRequest(const std::string &path, const std::string &body = "",
                   const cpr::Header &header = {}) {
    if (!initialized_) {
      ROS_ERROR("%s: not initialized.", this->getName().c_str());
      return;
    }
    if (!serverName) {
      ROS_ERROR("Server URL not set.");
      return;
    }
    request_ = cpr::GetCallback(
        [this](const cpr::Response &response) {
          onResponseReceived_(response);
        },
        cpr::Url{serverName.get() + path}, cpr::Body{{body}},
        cpr::Header{header}, cpr::Timeout{timeout.get()});
  }

 private:
  bool initialized_;
  std::future<void> request_;
};
}  // namespace client_bases
}  // namespace smacc