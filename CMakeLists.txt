cmake_minimum_required(VERSION 3.16)
project(GetPcmData LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 自动生成 moc、rcc、ui
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

# 查找 Qt6 模块
find_package(Qt6 REQUIRED COMPONENTS Core Gui Widgets SerialPort Multimedia Charts)

# 可执行文件
add_executable(GetPcmData
    WIN32
    main.cpp
    mainwindow.cpp


    # 如果使用 Designer UI
)

# 链接 Qt 库
target_link_libraries(GetPcmData PRIVATE
    Qt6::Core
    Qt6::Gui
    Qt6::Widgets
    Qt6::SerialPort
    Qt6::Multimedia
    Qt6::Charts
)
