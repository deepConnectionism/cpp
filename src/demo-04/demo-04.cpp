#include <iostream>
#include <thread>
#include <queue>
#include <mutex>
#include <condition_variable>

std::queue<int> buffer;             // 共享缓冲区
std::mutex mtx;                     // 互斥锁
std::condition_variable cv;         // 条件变量
bool done = false;                  // 生产结束标记

// 生产者线程
void producer() {
    for (int i = 1; i <= 5; i++) {
        {
            std::lock_guard<std::mutex> lock(mtx);
            buffer.push(i);
            std::cout << "Produced: " << i << std::endl;
        }
        cv.notify_one(); // 通知消费者有新数据
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    {   // 生产结束，通知消费者
        std::lock_guard<std::mutex> lock(mtx);
        done = true;
    }
    cv.notify_all();
}

// 消费者线程
void consumer() {
    while (true) {
        std::unique_lock<std::mutex> lock(mtx);
        cv.wait(lock, []{ return !buffer.empty() || done; });

        while (!buffer.empty()) {
            int item = buffer.front();
            buffer.pop();
            std::cout << "Consumed: " << item << std::endl;
        }

        if (done) break;
    }
}

int main() {
    std::thread t1(producer);
    std::thread t2(consumer);

    t1.join();
    t2.join();

    std::cout << "All done!" << std::endl;
    return 0;
}
