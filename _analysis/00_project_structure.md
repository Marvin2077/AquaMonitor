# 项目结构分析

## 基本信息

| 项目 | 详情 |
|------|------|
| 项目路径 | `g:/AquaMonitor/AquaMonitor` |
| 构建系统 | PlatformIO + CMake（双支持） |
| 目标平台 | ESP32（Arduino 框架） |
| 语言 | C / C++（.c、.cpp 混合） |
| 分析日期 | 2026-02-22 |

---

## 完整目录树

```
AquaMonitor/
├── .claude/
│   └── settings.json
├── .gitattributes
├── .gitignore
├── .vscode/
│   ├── c_cpp_properties.json
│   ├── extensions.json
│   ├── launch.json
│   └── settings.json
├── CMakeLists.txt                  ← 顶层 CMake 配置
├── platformio.ini                  ← PlatformIO 构建配置
├── sdkconfig.idf                   ← ESP-IDF SDK 配置
├── _analysis/                      ← 分析文档（本目录）
│   └── 00_project_structure.md
├── include/                        ← 头文件目录
│   ├── .idea/                      （JetBrains IDE 元数据，非代码）
│   ├── ad5940.h
│   ├── ad5941_board_glue.h
│   ├── ad5941PlatformCfg.h
│   ├── ads124s08_board_glue.h
│   ├── ads124s08_drv.h
│   ├── bt_service.h
│   ├── Conductivity_service.h
│   ├── freecl_service.h
│   ├── host_link.h
│   ├── mux_iface.h
│   ├── ph_service.h
│   ├── spi_hal.h
│   ├── storage_manager.h
│   └── temp_service.h
├── src/                            ← 源代码目录
│   ├── .idea/                      （JetBrains IDE 元数据，非代码）
│   ├── CMakeLists.txt              ← src 子目录 CMake 配置
│   ├── ad5940.c
│   ├── ad5941_board_glue.cpp
│   ├── ad5941PlatformCfg.cpp
│   ├── ads124s08_board_glue.cpp
│   ├── ads124s08_drv.cpp
│   ├── bt_service.cpp
│   ├── Conductivity_service.cpp
│   ├── freecl_service.cpp
│   ├── host_link.cpp
│   ├── main.cpp                    ← 程序入口
│   ├── mux_iface.cpp
│   ├── ph_service.cpp
│   ├── spi_hal.cpp
│   ├── storage_manager.cpp
│   └── temp_service.cpp
├── lib/
│   └── README                      ← PlatformIO 库占位说明
└── test/
    ├── Code_Structure              ← 测试/结构说明文件
    └── README
```

---

## 分类识别

### 1. 主要源代码目录

| 目录 | 文件类型 | 说明 |
|------|----------|------|
| `src/` | `.c` / `.cpp` | 全部实现文件，含程序入口 `main.cpp` |
| `include/` | `.h` | 全部头文件（公共接口声明） |

### 2. 驱动 / 外设模块

按功能分组：

#### ADC / 阻抗测量芯片
| 模块 | 头文件 | 源文件 | 说明 |
|------|--------|--------|------|
| AD5940 驱动 | `ad5940.h` | `ad5940.c` | ADI AD5940 阻抗分析仪驱动（纯 C） |
| AD5941 板级胶合层 | `ad5941_board_glue.h` | `ad5941_board_glue.cpp` | AD5941 硬件适配（ESP32 平台） |
| AD5941 平台配置 | `ad5941PlatformCfg.h` | `ad5941PlatformCfg.cpp` | AD5941 初始化参数配置 |
| ADS124S08 驱动 | `ads124s08_drv.h` | `ads124s08_drv.cpp` | TI ADS124S08 精密 ADC 驱动 |
| ADS124S08 板级胶合层 | `ads124s08_board_glue.h` | `ads124s08_board_glue.cpp` | ADS124S08 硬件适配 |

#### 硬件抽象层（HAL）
| 模块 | 头文件 | 源文件 | 说明 |
|------|--------|--------|------|
| SPI HAL | `spi_hal.h` | `spi_hal.cpp` | SPI 总线硬件抽象层 |
| 多路复用器接口 | `mux_iface.h` | `mux_iface.cpp` | 模拟多路复用器控制接口 |

#### 测量服务层
| 模块 | 头文件 | 源文件 | 说明 |
|------|--------|--------|------|
| pH 服务 | `ph_service.h` | `ph_service.cpp` | pH 值测量与计算 |
| 电导率服务 | `Conductivity_service.h` | `Conductivity_service.cpp` | 电导率测量与计算 |
| 游离氯服务 | `freecl_service.h` | `freecl_service.cpp` | 游离氯浓度测量 |
| 温度服务 | `temp_service.h` | `temp_service.cpp` | 温度测量服务 |

#### 系统服务
| 模块 | 头文件 | 源文件 | 说明 |
|------|--------|--------|------|
| 蓝牙服务 | `bt_service.h` | `bt_service.cpp` | BLE 通信服务 |
| 主机通信链路 | `host_link.h` | `host_link.cpp` | 上位机通信协议 |
| 存储管理器 | `storage_manager.h` | `storage_manager.cpp` | 数据持久化存储管理 |

### 3. 配置文件

| 文件 | 类型 | 说明 |
|------|------|------|
| `platformio.ini` | PlatformIO 配置 | 板型、框架、编译参数等 |
| `CMakeLists.txt` | CMake 顶层配置 | 顶层构建描述 |
| `src/CMakeLists.txt` | CMake 子目录配置 | src 目录源文件列表 |
| `sdkconfig.idf` | ESP-IDF SDK 配置 | ESP32 SDK 功能开关 |
| `.vscode/c_cpp_properties.json` | VS Code C/C++ 配置 | IntelliSense 路径配置 |
| `.vscode/launch.json` | VS Code 调试配置 | 调试启动参数 |
| `.vscode/settings.json` | VS Code 工作区配置 | 编辑器设置 |
| `.vscode/extensions.json` | VS Code 推荐扩展 | 推荐插件列表 |

### 4. 文档 / 说明文件

| 文件 | 说明 |
|------|------|
| `lib/README` | PlatformIO lib 目录使用说明（框架自带） |
| `test/README` | 测试目录说明 |
| `test/Code_Structure` | 代码结构说明文档 |

---

## 架构概览

```
                        main.cpp
                           │
           ┌───────────────┼───────────────┐
           │               │               │
      测量服务层        系统服务层       通信层
   ┌───────────────┐  ┌──────────┐  ┌──────────────┐
   │ ph_service    │  │ storage  │  │ bt_service   │
   │ Conductivity  │  │ _manager │  │ host_link    │
   │ freecl_service│  └──────────┘  └──────────────┘
   │ temp_service  │
   └───────┬───────┘
           │
       硬件抽象层
   ┌───────────────┐
   │  spi_hal      │
   │  mux_iface    │
   └───────┬───────┘
           │
       芯片驱动层
   ┌──────────────────────────┐
   │ AD5940/AD5941 (阻抗/电化学)│
   │ ADS124S08 (精密ADC)       │
   └──────────────────────────┘
```

---

## 推测项目功能

本项目为**水质监测仪**嵌入式固件，运行于 ESP32，能够测量：
- **pH 值**（电化学法，通过 ADS124S08 ADC）
- **电导率**（AC 阻抗法，通过 AD5940/AD5941）
- **游离氯**（电化学法）
- **温度**（辅助参数，用于温度补偿）

数据可通过 **BLE 蓝牙**或**有线主机链路**传输至上位机，并支持本地**存储管理**。
