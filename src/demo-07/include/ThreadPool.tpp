#ifndef THREADPOOL_TPP
#define THREADPOOL_TPP

#include "ThreadPool.h"

// 任务提交模板方法实现
template<class F, class... Args>
auto ThreadPool::submit(F&& f, Args&&... args) -> std::future<decltype(f(args...))> {
     // 推导返回类型
    using return_type = decltype(f(args...));

    // 封装任务为packaged_task
    auto taskPtr = std::make_shared<std::packaged_task<return_type()>>(
        std::bind(std::forward<F>(f), std::forward<Args>(args)...)
    );

    // 获取future对象
    std::future<return_type> res = taskPtr->get_future();

    // 临界区：提交任务
    {
        std::unique_lock<std::mutex> lock(mtx);
        if (stop) throw std::runtime_error("ThreadPool already stopped");

        // 将任务包装为void()类型
        tasks.emplace([taskPtr]() { (*taskPtr)(); });
    }

    // 通知一个等待线程
    cv.notify_one();
    return res;
}

#endif
