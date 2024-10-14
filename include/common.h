#ifndef __SUPERPOSITION_COMMON_H__
#define __SUPERPOSITION_COMMON_H__
#include <atomic>
#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <chrono>
#include <iostream>
#include <thread>

using namespace boost;

namespace superposition::common {

class AbstractTask {
   protected:
    std::thread _thread;
    std::atomic<bool> _is_started;

   public:
    virtual void process() {
        std::cout << "call Task::process" << std::endl;
    }
    AbstractTask() : _is_started(false) {
    }

    virtual ~AbstractTask() {
    }

    virtual void start() {
        if (_is_started.load()) {
            return;
        }

        _is_started.store(true);
        _thread = std::thread(&AbstractTask::process, this);
    }

    virtual void stop() {
        if (!_is_started.load()) {
            return;
        }

        _is_started.store(false);
    }

    bool is_started() {
        return _is_started.load();
    }
    void join() {
        if (_thread.joinable()) {
            _thread.join();
        }
    }
};

class SimpleScheduler {
   public:
    SimpleScheduler(asio::io_context &io, int interval,
                    std::function<void()> task)
        : _timer(io, std::chrono::milliseconds(interval)),
          _interval(interval),
          _task(task) {
    }

    virtual ~SimpleScheduler() {
    }

    void execute(const system::error_code &e) {
        if (e == boost::asio::error::operation_aborted) {
            return;
        }

        _task();
        reset_timer();
    }

    void start() {
        _timer.async_wait(boost::bind(&SimpleScheduler::execute, this,
                                      asio::placeholders::error));
    }

   private:
    boost::asio::steady_timer _timer;
    int _interval;
    std::function<void()> _task;

    void reset_timer() {
        _timer.expires_after(std::chrono::milliseconds(_interval));
        _timer.async_wait(boost::bind(&SimpleScheduler::execute, this,
                                      asio::placeholders::error));
    }
};

template <typename T>
class MessageBuffer {
   public:
    MessageBuffer(size_t size) : buffer(size), head(0), tail(0), full(false) {
    }

    void push(T item) {
        std::lock_guard<std::mutex> lock(mutex);
        buffer[head] = item;
        if (full) {
            tail = (tail + 1) % buffer.size();
        }
        head = (head + 1) % buffer.size();
        full = head == tail;
    }

    bool pop(T &out) {
        std::lock_guard<std::mutex> lock(mutex);
        if (is_empty()) {
            return false;
        }
        out = buffer[tail];
        full = false;
        tail = (tail + 1) % buffer.size();
        return true;
    }

    bool is_empty() const {
        return (!full && (head == tail));
    }

    bool is_full() const {
        return full;
    }

   private:
    std::vector<T> buffer;
    size_t head;
    size_t tail;
    bool full;
    std::mutex mutex;
};

}  // namespace superposition::common

#endif