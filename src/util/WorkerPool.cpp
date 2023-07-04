#include "WorkerPool.h"
#include <thread>

//https://stackoverflow.com/questions/15752659/thread-pooling-in-c11
//https://stackoverflow.com/questions/22030027/c11-dynamic-threadpool/32593766#32593766
//std::promise https://stackoverflow.com/questions/11004273/what-is-stdpromise https://en.cppreference.com/w/cpp/thread/promise

WorkerPool::WorkerPool(bool multithreaded): jobs_(), shutdown_(false)
{
    const unsigned int numThreads = multithreaded ? std::thread::hardware_concurrency() : 1;

    threads_.resize(numThreads);
    if(numThreads > 1)
    {
        for(unsigned int i = 0; i < numThreads; i++)
        {
            threads_[i] = std::thread(&WorkerPool::threadLoop, this);
        }
    }
}

WorkerPool::~WorkerPool()
{
    shutdown_ = true;

    for(unsigned int i = 0; i < getNumThreads(); i++)
    {
        run<void>([] {});
    }

    for(std::thread& thread : threads_)
    {
        thread.join();
    }
}

void WorkerPool::threadLoop()
{
    while (!shutdown_)
    {
        std::function<void()> job;
        {
            std::unique_lock<std::mutex> lock(queueMutex_);
            mutexCondition_.wait(lock, [this] { return !jobs_.empty(); });

            job = jobs_.front();
            jobs_.pop();
        }
        job();
    }
}

template <>
std::future<void> WorkerPool::run<void>(const std::function<void()>& job)
{
    std::shared_ptr<std::promise<void>> promise = std::make_shared<std::promise<void>>();
    std::future<void> future = promise->get_future();

    if(getNumThreads() > 1)
    {
        {
            std::unique_lock<std::mutex> lock(queueMutex_);
            jobs_.emplace([=] { job(); promise->set_value(); });
        }
        mutexCondition_.notify_one();
    }
    else
    {
        job();
        promise->set_value();
    }

    return future;
}
