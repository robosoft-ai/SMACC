#pragma once

#include <boost/circular_buffer.hpp>
#include <condition_variable>
#include <functional>
#include <mutex>
#include <queue>
#include <thread>

namespace smacc {
class ISmaccThreadable {
 public:
  /**
   * Layout of this threadpool inspired from the SO answer at
   * https://stackoverflow.com/a/32593825/3081770
   */

  ISmaccThreadable(const uint32_t& num_threads = 1,
                   const uint32_t& queue_size = 1);

 protected:
  template <typename F, typename... Args>
  void runOnThread(F f, Args&... args) {
    {
      std::unique_lock<std::mutex> lock{thread_mtx_};
      job_queue_.push(std::bind(f, std::forward<Args>(args)...));
    }
    if (!busy()) jobs_waiting_.notify_one();
  }

  bool busy();
  void stop();

 private:
  void workerLoop();

  std::mutex thread_mtx_;
  std::condition_variable jobs_waiting_;
  std::vector<std::thread> threads_;
  std::queue<std::function<void()>,
             boost::circular_buffer<std::function<void()>>>
      job_queue_;

  bool should_terminate_;
};
}  // namespace smacc
