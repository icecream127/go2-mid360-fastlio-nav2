#pragma once
#include <condition_variable>
#include <exception>
#include <functional>
#include <mutex>
#include <optional>
#include <thread>

namespace go2_real_localization {
// At most one running job/result. Input backlog stays in the node's latest pair.
template<class Job, class Result> class RegistrationWorker {
public:
  struct Completion { std::optional<Result> value; std::exception_ptr error; };
  explicit RegistrationWorker(std::function<Result(const Job &)> work)
      : work_(std::move(work)), thread_([this] { run(); }) {}
  ~RegistrationWorker() {
    { std::lock_guard<std::mutex> lock(mutex_); stopping_ = true; }
    condition_.notify_one(); thread_.join();
  }
  bool busy() const { std::lock_guard<std::mutex> lock(mutex_); return busy_; }
  bool submit(Job job) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (busy_ || stopping_) return false;
    job_ = std::move(job); busy_ = true; condition_.notify_one(); return true;
  }
  std::optional<Completion> take() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!completion_) return std::nullopt;
    auto result = std::move(completion_); completion_.reset(); busy_ = false; return result;
  }
private:
  void run() {
    for (;;) {
      std::unique_lock<std::mutex> lock(mutex_);
      condition_.wait(lock, [this] { return stopping_ || job_.has_value(); });
      if (stopping_) return;
      Job job = std::move(*job_); job_.reset(); lock.unlock();
      Completion completed;
      try { completed.value = work_(job); } catch (...) { completed.error = std::current_exception(); }
      lock.lock(); completion_ = std::move(completed);
    }
  }
  std::function<Result(const Job &)> work_;
  mutable std::mutex mutex_;
  std::condition_variable condition_;
  bool stopping_ = false, busy_ = false;
  std::optional<Job> job_;
  std::optional<Completion> completion_;
  std::thread thread_;
};
}  // namespace go2_real_localization
