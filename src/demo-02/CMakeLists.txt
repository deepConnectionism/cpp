add_executable(demo-02 demo-02.cpp)
target_link_libraries(demo-02 PRIVATE 
    Threads::Threads  # 继承公共配置
    # 可添加其他私有依赖
)