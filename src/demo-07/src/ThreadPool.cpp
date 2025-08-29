#include "ThreadPool.h"

// 构造函数实现
ThreadPool::ThreadPool(size_t threadCount) : stop(false)  // 初始化停止标志
{
     // 创建工作线程
    for (size_t i = 0; i < threadCount; ++i) {
        workers.emplace_back([this] {
            // 线程任务循环
            while (true) {
                std::function<void()> task;

                 // 临界区：获取任务
                {
                    std::unique_lock<std::mutex> lock(this->mtx);
                    this->cv.wait(lock, [this] { return stop || !tasks.empty(); });

                    // 停止条件检查
                    if (stop && tasks.empty()) return;

                    // 移动语义获取任务
                    task = std::move(tasks.front());
                    tasks.pop();
                }

                task();  // 执行任务
            }
        });
    }
}

// 析构函数实现
ThreadPool::~ThreadPool() {
    // 触发停止标志
    {
        std::unique_lock<std::mutex> lock(mtx);
        stop = true;
    }

     // 通知所有线程
    cv.notify_all();

    // 等待线程结束
    for (std::thread &worker : workers) {
        if (worker.joinable()) worker.join();
    }
}
