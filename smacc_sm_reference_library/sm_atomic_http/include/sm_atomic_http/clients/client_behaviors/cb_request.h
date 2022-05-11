#include <smacc/smacc.h>

namespace sm_atomic_http {

class CbRequest : public smacc::ISmaccClientBehavior {
 public:
  void onEntry() override {
    ClHttp* cl_http;
    this->requiresClient<ClHttp>(cl_http);
    cl_http->onResponseReceived(&CbRequest::onResponseReceived, this);

    cl_http->makeGetRequest("/");
  }

  void onResponseReceived(const cpr::Response& response) {
    if (response.status_code == 200) {
      ROS_INFO("Good response received");
      exitState();
    } else {
      ROS_WARN("Response error: %s", response.error.message.c_str());
    }
  }

  template <typename TOrthogonal, typename TSourceObject>
  void onOrthogonalAllocation() {
    exitState = [=]() {
      this->postEvent<EvHttp<TSourceObject, TOrthogonal>>();
    };
  }

 private:
  std::function<void()> exitState;
};

}  // namespace sm_atomic_http