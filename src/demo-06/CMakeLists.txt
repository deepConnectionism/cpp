add_executable(demo-06 demo-06.cpp)
target_link_libraries(demo-06 PRIVATE 
    Threads::Threads  # 继承公共配置
    # 可添加其他私有依赖
)