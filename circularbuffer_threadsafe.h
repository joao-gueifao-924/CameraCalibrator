#ifndef CIRCULARBUFFER_THREADSAFE_H
#define CIRCULARBUFFER_THREADSAFE_H

#include <condition_variable>
#include <mutex>
#include <boost/circular_buffer.hpp>

template<typename T>
class circular_buffer_thread_safe
{
public:
    typedef std::unique_lock<std::mutex> unilock_mutex;

    circular_buffer_thread_safe(size_t capacity)
         : buffer_(capacity) {}

    void push(const T &item)
    {
        unilock_mutex lock(door_);
        buffer_.push_back(item);

        // manual unlocking is done before notifying, to avoid waking up
        // the waiting thread only to block again (see notify_one for details)
        lock.unlock();
        waiting_condition_.notify_one();
    }

    bool pop(T& item)
    {
        unilock_mutex lock(door_);
        waiting_condition_.wait(lock, [this]() { return !buffer_.empty() || !should_wait_to_pop_; });

        if (buffer_.empty()) return false;

        item = buffer_.front();
        buffer_.pop_front();

        return true;
    }

    void should_wait_to_pop(bool val)
    {
        should_wait_to_pop_ = val;
        waiting_condition_.notify_one();
    }

    bool is_empty() { return buffer_.empty(); }

private:
    boost::circular_buffer<T> buffer_;
    std::mutex door_;
    std::condition_variable waiting_condition_;
    std::atomic<bool> should_wait_to_pop_{ true };
};

#endif // CIRCULARBUFFER_THREADSAFE_H


