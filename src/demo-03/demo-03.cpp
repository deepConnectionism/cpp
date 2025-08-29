#include <iostream>
#include <thread>
#include <mutex>

int counter = 0;
std::mutex mtx;  // 全局互斥锁

void worker(int x) {
    for (int i = 0; i < 5; i++) {
        std::lock_guard<std::mutex> lock(mtx); // 自动上锁和解锁
        counter += x;
        std::cout << "Thread " << std::this_thread::get_id()
                  << " add " << x
                  << " -> counter = " << counter << std::endl;
    }
}

int main() {
    std::thread t1(worker, 10);
    std::thread t2(worker, 20);
    t1.join();
    t2.join();
    std::cout << "Final counter = " << counter << std::endl;
    return 0;
}
