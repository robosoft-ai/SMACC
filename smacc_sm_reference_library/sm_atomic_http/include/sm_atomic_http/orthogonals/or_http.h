#include <smacc/smacc.h>

namespace sm_atomic_http {
class OrHttp : public smacc::Orthogonal<OrHttp> {
 public:
  virtual void onInitialize() override {
    auto client = this->createClient<ClHttp>("www.example.com", 2000);
    client->initialize();
  }
};
}  // namespace sm_atomic_http
