#include <smacc/smacc_client_behavior.h>
#include <sm_ridgeback_floor_coverage_static_1/clients/cl_service3/cl_service3.h>

namespace sm_ridgeback_floor_coverage_static_1
{
namespace cl_service3
{
enum class Service3Command
{
  SERVICE3_ON,
  SERVICE3_OFF
};

class CbService3 : public smacc::SmaccClientBehavior
{
private:
  ClService3 *serviceClient_;
  Service3Command value_;

public:
  CbService3(Service3Command value)
  {
    value_ = value;
  }

  virtual void onEntry() override
  {
    this->requiresClient(serviceClient_);

    std_srvs::SetBool req;
    if (value_ == Service3Command::SERVICE3_ON)
      req.request.data = true;
    else
      req.request.data = false;

    serviceClient_->call(req);
  }
};
} // namespace cl_service3
} // namespace sm_ridgeback_floor_coverage_static_1
