#pragma once
#include <smacc/smacc_client_behavior.h>

namespace smacc {
template <typename TService>
class CbServiceServerCallbackBase : public smacc::SmaccClientBehavior {
 public:
  virtual void onEntry() override {
    this->requiresClient(attachedClient_);
    attachedClient_->onServiceRequestReceived(
        &CbServiceServerCallbackBase::onServiceRequestReceived, this);
  }

  virtual bool onServiceRequestReceived(typename TService::Request& req,
                                        std::shared_ptr<typename TService::Response> res) = 0;

 protected:
  smacc::client_bases::SmaccServiceServerClient<TService>* attachedClient_ =
      nullptr;
};
}  // namespace smacc
