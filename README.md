这是 vscode 的 C++ 项目模板，主要讲解了多线程和多线程

使用分层CMake结构管理多目标，以下是具体实施方案：

项目结构调整建议

    ├── CMakeLists.txt          # 根配置
    └── src/
        ├── CMakeLists.txt      # 公共配置
        ├── demo-01/
        │   ├── CMakeLists.txt  # 独立配置
        │   └── demo-01.cpp
        └── demo-02/
            ├── CMakeLists.txt  # 独立配置
            └── demo-02.cpp

优势说明：

1. 根配置管理全局设置（C++标准/编译选项）
2. 公共配置层集中管理共享依赖（如线程库）
3. 每个可执行文件：
    - 拥有独立目录和CMakeLists.txt
    - 可定义私有依赖和特殊配置
    - 通过target_link_libraries继承公共依赖
4. 新项目只需：
    - 创建新目录和CMakeLists.txt
    - 在src/CMakeLists.txt添加add_subdirectory

执行构建命令保持不变：

    mkdir -p build && cd build
    cmake .. 
    make

每次调试可以点左下角的运行按钮。并且改变 `launch.json ： "program": "${workspaceFolder}/build/src/demo-02/demo-02"`,  的程序，在 cpp 中加断点即可。