#include <smacc/client_bases/smacc_http_client.h>
#include <smacc/smacc.h>

namespace sm_atomic_http {

template <typename TSource, typename TOrthogonal>
struct EvHttp : sc::event<EvHttp<TSource, TOrthogonal>> {};

class ClHttp : public smacc::client_bases::SmaccHttpClient {
 public:
  ClHttp(const std::string& server, const int& timeout)
      : smacc::client_bases::SmaccHttpClient(server, timeout) {}
};
}  // namespace sm_atomic_http
