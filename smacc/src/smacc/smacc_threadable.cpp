#include <smacc/smacc_threadable.h>

namespace smacc {
ISmaccThreadable::ISmaccThreadable(const uint32_t& num_threads,
                                   const uint32_t& queue_size)
    : job_queue_{boost::circular_buffer<std::function<void()>>{queue_size}},
      should_terminate_{false} {
  threads_.resize(num_threads);
  for (uint32_t i = 0; i < num_threads; ++i) {
    threads_.at(i) = std::thread(&ISmaccThreadable::workerLoop, this);
  }
}

bool ISmaccThreadable::busy() {
  bool busy;
  {
    std::unique_lock<std::mutex> lock{thread_mtx_};

    busy = job_queue_.empty();
  }
  return busy;
}

void ISmaccThreadable::stop() {
  {
    std::unique_lock<std::mutex> lock{thread_mtx_};
    should_terminate_ = true;
  }
  jobs_waiting_.notify_all();

  for (auto& thread : threads_) {
    thread.join();
  }
  threads_.clear();
}

void ISmaccThreadable::workerLoop() {
  while (true) {
    std::function<void()> job;
    {
      std::unique_lock<std::mutex> lock{thread_mtx_};

      jobs_waiting_.wait(
          lock, [this] { return !job_queue_.empty() || should_terminate_; });
      if (should_terminate_) return;

      job = job_queue_.front();
      job_queue_.pop();
    }
    job();
  }
}
}  // namespace smacc
