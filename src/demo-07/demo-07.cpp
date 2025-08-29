#include "ThreadPool.h"
#include <iostream>
#include <cmath>

int heavyTask(int id, int n) {
    double result = 0;
    for (int i = 0; i < n * 1000000; i++) {
        result += std::sin(i) * std::cos(i);
    }
    std::cout << "Task " << id << " finished\n";
    return static_cast<int>(result);
}

int main() {
    ThreadPool pool(4);

    std::vector<std::future<int>> results;
    for (int i = 0; i < 8; i++) {
        results.push_back(pool.submit(heavyTask, i, 5));
    }

    for (auto &f : results) {
        std::cout << "Result: " << f.get() << std::endl;
    }

    std::cout << "All tasks done!\n";
    return 0;
}
