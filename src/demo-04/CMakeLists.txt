add_executable(demo-04 demo-04.cpp)
target_link_libraries(demo-04 PRIVATE 
    Threads::Threads  # 继承公共配置
    # 可添加其他私有依赖
)