include_directories(include)
add_executable(demo-07
    demo-07.cpp
    src/ThreadPool.cpp
)
target_link_libraries(demo-07 PRIVATE 
    Threads::Threads  # 继承公共配置
    # 可添加其他私有依赖
)