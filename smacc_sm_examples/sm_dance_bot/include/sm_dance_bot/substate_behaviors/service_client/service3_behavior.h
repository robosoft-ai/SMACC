#include <smacc/smacc_substate_behavior.h>
#include <sm_dance_bot/substate_behaviors/service_client/service3_client.h>

enum class Service3Command {SERVICE3_ON, SERVICE3_OFF};

class Service3Behavior : public smacc::SmaccSubStateBehavior
{
private:
  ServiceClient3 *serviceClient_;
  Service3Command value_;

public:
  Service3Behavior(Service3Command value)
  {
    value_ = value;
  }

  virtual void onEntry() override
  {
    this->requiresClient(serviceClient_);
    
    std_srvs::SetBool req;
    if(value_ == Service3Command::SERVICE3_ON)
      req.request.data = true;
    else
      req.request.data = false;
    
    serviceClient_->call(req);
  }
};