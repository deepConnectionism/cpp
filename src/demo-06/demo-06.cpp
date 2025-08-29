#include <iostream>
#include <thread>
#include <queue>
#include <mutex>
#include <condition_variable>
#include <vector>
#include <chrono>
#include <random>

std::queue<int> buffer;            
std::mutex mtx;                    
std::condition_variable cv;        
bool done = false;                 

// 随机数模拟"读取数据耗时"
int random_delay() {
    static thread_local std::mt19937 gen(std::random_device{}());
    std::uniform_int_distribution<int> dist(100, 500);
    return dist(gen);
}

// 多个生产者：模拟数据加载
void producer(int id, int count) {
    for (int i = 1; i <= count; i++) {
        std::this_thread::sleep_for(std::chrono::milliseconds(random_delay()));
        {
            std::lock_guard<std::mutex> lock(mtx);
            buffer.push(i + id * 1000); // 给不同生产者加偏移量
            std::cout << "[Producer " << id << "] loaded sample " << (i + id * 1000) << std::endl;
        }
        cv.notify_one();
    }
}

// 单个消费者：模拟 GPU 训练线程
void consumer() {
    while (true) {
        std::unique_lock<std::mutex> lock(mtx);
        cv.wait(lock, []{ return !buffer.empty() || done; });

        while (!buffer.empty()) {
            int data = buffer.front();
            buffer.pop();
            lock.unlock();  // 解锁，允许生产者继续填充
            std::cout << "    [Consumer] training on sample " << data << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(300)); // 模拟计算耗时
            lock.lock();
        }

        if (done && buffer.empty()) break;
    }
}

int main() {
    int producer_count = 3; // 三个生产者
    int samples_per_producer = 5;

    std::vector<std::thread> producers;
    for (int i = 0; i < producer_count; i++) {
        producers.emplace_back(producer, i, samples_per_producer);
    }

    std::thread gpu(consumer);

    for (auto &p : producers) p.join();

    { std::lock_guard<std::mutex> lock(mtx); done = true; }
    cv.notify_all();

    gpu.join();
    std::cout << "All training finished!" << std::endl;
    return 0;
}
