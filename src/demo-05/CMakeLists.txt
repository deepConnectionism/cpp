add_executable(demo-05 demo-05.cpp)
target_link_libraries(demo-05 PRIVATE 
    Threads::Threads  # 继承公共配置
    # 可添加其他私有依赖
)