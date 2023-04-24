#pragma once

#include <smacc/smacc_client.h>
#include <smacc/smacc_signal.h>
#include <std_srvs/Empty.h>

#include <boost/optional/optional_io.hpp>

namespace smacc {
namespace client_bases {
template <typename TService>
class SmaccServiceServerClient : public smacc::ISmaccClient {
  using TServiceRequest = typename TService::Request;
  using TServiceResponse = typename TService::Response;

 public:
  boost::optional<std::string> serviceName_;
  SmaccServiceServerClient() { initialized_ = false; }
  SmaccServiceServerClient(std::string service_name) {
    serviceName_ = service_name;
    initialized_ = false;
  }

  virtual ~SmaccServiceServerClient() { server_.shutdown(); }

  smacc::SmaccSignal<void(TServiceRequest&, std::shared_ptr<TServiceResponse>)>
      onServiceRequestReceived_;

  template <typename T>
  boost::signals2::connection onServiceRequestReceived(
      void (T::*callback)(TServiceRequest&, std::shared_ptr<TServiceResponse>),
      T* object) {
    return this->getStateMachine()->createSignalConnection(
        onServiceRequestReceived_, callback, object);
  }

  virtual void initialize() override {
    if (!initialized_) {
      if (!serviceName_) {
        ROS_ERROR("service server with no service name set. Skipping.");
      } else {
        ROS_INFO_STREAM("[" << this->getName()
                            << "] Client Service: " << serviceName_);

        server_ = nh_.advertiseService(
            *serviceName_, &SmaccServiceServerClient<TService>::serviceCallback,
            this);
        this->initialized_ = true;
      }
    }
  }

 protected:
  ros::NodeHandle nh_;

 private:
  bool serviceCallback(TServiceRequest& req, TServiceResponse& res) {
    std::shared_ptr<TServiceResponse> response{new TServiceResponse};
    onServiceRequestReceived_(req, response);

    res = *response;
    return true;
  }
  ros::ServiceServer server_;
  bool initialized_;
};
}  // namespace client_bases
}  // namespace smacc
