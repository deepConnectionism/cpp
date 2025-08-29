#include <iostream>
#include <thread>

int counter = 0;

void worker(int x) {
    counter += x;
    std::cout << "Thread add " << x 
              << " -> counter = " << counter << std::endl;
}

int main() {
    std::thread t1(worker, 10);
    std::thread t2(worker, 20);
    t1.join();
    t2.join();
    std::cout << "Final counter = " << counter << std::endl;
    return 0;
}
