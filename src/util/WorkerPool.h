#pragma once

#include <mutex>
#include <queue>
#include <vector>
#include <thread>
#include <condition_variable>
#include <functional>
#include <future>

class WorkerPool
{
public:
    explicit WorkerPool(bool multithreaded = true);

    ~WorkerPool();

    template<typename T>
    std::future<T> run(const std::function<T()>& job)
    {
        std::shared_ptr<std::promise<T>> promise = std::make_shared<std::promise<T>>();
        std::future<T> future = promise->get_future();

        if(getNumThreads() > 1)
        {
            {
                std::unique_lock<std::mutex> lock(queueMutex_);
                jobs_.emplace([=] { promise->set_value(job()); });
            }
            mutexCondition_.notify_one();
        }
        else
        {
            promise->set_value(job());
        }

        return future;
    }

    template<typename T>
    void waitFor(const std::vector<std::future<T>>& jobs)
    {
        for(const std::future<T>& job : jobs)
            job.wait();
    }

    unsigned int getNumThreads() const { return threads_.size(); }

private:
    void threadLoop();

    std::mutex queueMutex_;
    std::condition_variable mutexCondition_;
    std::vector<std::thread> threads_;
    std::queue<std::function<void()>> jobs_;
    bool shutdown_;
};

template <>
std::future<void> WorkerPool::run<void>(const std::function<void()>& job);
