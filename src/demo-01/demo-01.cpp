#include <iostream>
#include <unistd.h>   // fork, getpid

int counter = 0;

int main() {
    pid_t pid = fork(); // 创建子进程

    if (pid == 0) { // 子进程
        counter += 10;
        std::cout << "Child process (pid=" << getpid()
                  << ") counter = " << counter << std::endl;
    } else { // 父进程
        counter += 20;
        std::cout << "Parent process (pid=" << getpid()
                  << ") counter = " << counter << std::endl;
    }
    return 0;
}