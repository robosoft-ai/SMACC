#include <smacc/smacc.h>

namespace sm_atomic_http {

class CbRequest : public smacc::ISmaccClientBehavior {
 public:
  void onEntry() override {
    ClHttp* cl_http;
    this->requiresClient<ClHttp>(cl_http);
    cl_http->onResponseReceived(&CbRequest::onResponseReceived, this);

    cl_http->makeRequest("/");
  }

  void onResponseReceived(
      const boost::beast::http::response<boost::beast::http::string_body>&
          response) {
    if (response.result_int() == 200) {
      ROS_INFO("Good response received");
      ROS_INFO_STREAM(response.body());
      ROS_INFO_STREAM(response.result_int());
      ROS_INFO_STREAM(response.base());
      exitState();
    } else {
      ROS_WARN("Response error: %s", response.reason());
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
