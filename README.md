# ING ASDM 模块采样PCM数据工具

## 简介

本仓库是[外设示例](https://github.com/ingchips/examples_peripherals)的演示工具，可以通过该工具使用串口收取外设ASDM的DEMO发送的PCM数据流，并展示音频数据流的波形，且可以实时通过PC的audio设备播放麦克风收取到的音频数据流。

## 如何编译

- 下载QT6安装工具，安装QT6工具

[官方地址](https://www.qt.io/download-dev#eval-form)

[中科大镜像](https://mirrors.ustc.edu.cn/qtproject/official_releases/online_installers/)

- 在工具中打开本仓库的QT工程（使用CMAKE进行工程管理）
- 编译工程
- 运行

## 获取release可执行软件包

在[release界面](https://github.com/ingchips/ASDM_test_tools/releases/tag/0.2)获取包含exe和动态链接库的压缩包https://github.com/ingchips/ASDM_test_tools/releases/download/0.2/INGCHIPS_TestAsdmTool.7z

解压运行，按照example的描述设定正确参数，开始测试。