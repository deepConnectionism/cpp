# 公共依赖配置
find_package(Threads REQUIRED)
set(THREADS_PREFER_PTHREAD_FLAG ON)
set(CMAKE_THREAD_PREFER_PTHREAD TRUE)

# 包含子项目
# 进程 vs 线程的差异（独立 vs 共享内存）
add_subdirectory(demo-01)
# 线程之间 共享内存
add_subdirectory(demo-02)
# 线程加锁版（使用 std::mutex）
add_subdirectory(demo-03)
# 生产者-消费者（条件变量版）
add_subdirectory(demo-04)
# 多生产者-多消费者
add_subdirectory(demo-05)
# 多生产者-消费者 是怎么用的（结合算法方向的场景）
add_subdirectory(demo-06)
# 线程池（Thread Pool）
add_subdirectory(demo-07)
