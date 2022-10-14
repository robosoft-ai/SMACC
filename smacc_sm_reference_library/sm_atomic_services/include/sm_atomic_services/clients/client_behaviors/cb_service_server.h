#include <smacc/client_behavior_bases/cb_service_server_callback_base.h>
#include <smacc/smacc_client_behavior.h>

namespace sm_atomic_services
{
    template <typename TSource, typename TOrthogonal>
    struct EvServiceRequestReceieved : sc::event<EvServiceRequestReceieved<TSource, TOrthogonal>> {};

    class CbServiceServer : public smacc::CbServiceServerCallbackBase<std_srvs::Empty>
    {
      public:
        bool onServiceRequestReceived(std_srvs::Empty::Request& req, std::shared_ptr<std_srvs::Empty::Response> res) override
        {
            requestReceived();
            // res->success = true;

            return true;
        }

        template <typename TOrthogonal, typename TSourceObject>
        void onOrthogonalAllocation()
        {
          this->requestReceived = [=]()
          {
            this->template postEvent<EvServiceRequestReceieved<TSourceObject, TOrthogonal>>();
          };
        }
      private:
        std::function<void()> requestReceived;
  };
}
