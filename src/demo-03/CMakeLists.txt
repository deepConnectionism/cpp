add_executable(demo-03 demo-03.cpp)
target_link_libraries(demo-03 PRIVATE 
    Threads::Threads  # 继承公共配置
    # 可添加其他私有依赖
)