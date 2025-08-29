cmake_minimum_required(VERSION 3.10)
project(Cpp LANGUAGES CXX)

# 全局设置
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra -g)

# 添加子目录
add_subdirectory(src)