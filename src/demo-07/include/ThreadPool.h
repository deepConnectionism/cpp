#ifndef THREADPOOL_H
#define THREADPOOL_H

#include <vector>
#include <queue>
#include <thread>
#include <future>
#include <mutex>
#include <condition_variable>
#include <functional>
#include <atomic>

// 线程池类声明
class ThreadPool {
public:
    // 构造函数，默认使用硬件并发线程数
    explicit ThreadPool(size_t threadCount = std::thread::hardware_concurrency());

    // 析构函数（负责资源回收）
    ~ThreadPool();

    // 任务提交模板方法
    // 参数：可调用对象及其参数
    // 返回：与任务返回值类型匹配的future对象
    template<class F, class... Args>
    auto submit(F&& f, Args&&... args) -> std::future<decltype(f(args...))>;

private:
    std::vector<std::thread> workers;        // 工作线程容器      
    std::queue<std::function<void()>> tasks;      // 任务队列 


    // 同步原语
    std::mutex mtx;    // 队列访问互斥锁
    std::condition_variable cv;     // 任务队列条件变量
    std::atomic<bool> stop;     // 停止标志（原子操作）
};

#include "ThreadPool.tpp"  // 模板成员函数的实现（见下）
#endif