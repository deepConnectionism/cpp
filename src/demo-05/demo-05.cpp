#include <iostream>
#include <thread>
#include <queue>
#include <mutex>
#include <condition_variable>
#include <vector>

std::queue<int> buffer;            
std::mutex mtx;                    
std::condition_variable cv;        
bool done = false;                 

// 生产者线程函数
void producer(int id, int count) {
    for (int i = 1; i <= count; i++) {
        {
            std::lock_guard<std::mutex> lock(mtx);
            buffer.push(i + id * 100); // 给不同生产者的数据加标识
            std::cout << "[Producer " << id << "] Produced: " << (i + id * 100) << std::endl;
        }
        cv.notify_one();
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
}

// 消费者线程函数
void consumer(int id) {
    while (true) {
        std::unique_lock<std::mutex> lock(mtx);
        cv.wait(lock, []{ return !buffer.empty() || done; });

        while (!buffer.empty()) {
            int item = buffer.front();
            buffer.pop();
            std::cout << "    [Consumer " << id << "] Consumed: " << item << std::endl;
        }

        if (done && buffer.empty()) break;
    }
}

int main() {
    int producer_count = 2;
    int consumer_count = 2;

    std::vector<std::thread> producers;
    std::vector<std::thread> consumers;

    // 启动生产者
    for (int i = 0; i < producer_count; i++) {
        producers.emplace_back(producer, i, 5);
    }

    // 启动消费者
    for (int i = 0; i < consumer_count; i++) {
        consumers.emplace_back(consumer, i);
    }

    // 等待生产者结束
    for (auto &p : producers) p.join();

    // 设置 done 并通知所有消费者退出
    {
        std::lock_guard<std::mutex> lock(mtx);
        done = true;
    }
    cv.notify_all();

    // 等待消费者结束
    for (auto &c : consumers) c.join();

    std::cout << "All done!" << std::endl;
    return 0;
}
