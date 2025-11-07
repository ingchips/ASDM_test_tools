cmake_minimum_required(VERSION 3.16)
project(INGCHIPS_TestAsdmTool LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 自动生成 moc、rcc、ui
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

# 查找 Qt6 模块
find_package(Qt6 REQUIRED COMPONENTS Core Gui Widgets SerialPort Multimedia Charts)

# 可执行文件
add_executable(INGCHIPS_TestAsdmTool
    WIN32
    main.cpp
    mainwindow.cpp
)

if(WIN32)
    target_sources(${PROJECT_NAME} PRIVATE
        Icon/appicon.rc
    )
    set_target_properties(${PROJECT_NAME} PROPERTIES
        WIN32_EXECUTABLE TRUE
        OUTPUT_NAME "INGCHIPS_TestAsdmTool"
    )
endif()

# 链接 Qt 库
target_link_libraries(INGCHIPS_TestAsdmTool PRIVATE
    Qt6::Core
    Qt6::Gui
    Qt6::Widgets
    Qt6::SerialPort
    Qt6::Multimedia
    Qt6::Charts
)
