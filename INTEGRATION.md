# 推理功能集成文档

## 集成概述

本文档描述了将模型推理功能集成到青龙ROS数据记录系统中的过程。我们从 `easy_inference_openloong.py` 中提取了核心功能，封装为一个完整的推理引擎模块，并集成到主系统中。

## 集成架构

集成架构主要包含以下部分：

1. **推理引擎模块** (`inference_engine.py`): 负责加载模型、获取观测数据、执行推理和发布控制指令
2. **配置模块扩展** (`config.py`): 增加推理相关参数支持
3. **UI界面扩展** (`main_window.py`): 新增推理标签页和相关操作界面
4. **启动脚本** (`run_inference.py`): 便于用户快速启动带推理功能的系统

## 核心功能

1. **模型加载与执行**
   - 支持ACT、扩散策略和PI0三种策略类型
   - 支持CUDA、CPU和MPS多种设备
   - 支持实时推理和单步推理

2. **ROS接口**
   - 接收机器人状态和相机图像
   - 发布机器人控制指令
   - 支持自定义话题配置

3. **UI交互**
   - 提供模型参数配置界面
   - 实时显示推理状态和性能
   - 支持录制推理过程数据

## 代码结构

```
ros_data_recorder/
├── ros_data_recorder/
│   ├── core/
│   │   ├── inference_engine.py  # 新增: 推理引擎核心模块
│   ├── ui/
│   │   ├── main_window.py       # 修改: 增加推理标签页
│   ├── config.py                # 修改: 增加推理参数
├── scripts/
│   ├── run_inference.py         # 新增: 推理启动脚本
├── requirements.txt             # 修改: 增加推理依赖
├── README.md                    # 修改: 增加推理说明
├── CHANGELOG.md                 # 新增: 变更日志
└── INTEGRATION.md               # 新增: 集成文档
```

## 集成流程

### 1. 推理引擎模块创建

从 `easy_inference_openloong.py` 中提取了以下核心组件：

- `PolicyFactory`: 负责根据策略类型创建不同的模型
- `RosOperator`: 改进为更通用的 `InferenceEngine` 类
- 观测数据处理和动作发布逻辑

### 2. 扩展配置支持

在配置模块中增加了以下参数：

- `--enable_inference`: 是否启用推理功能
- `--policy_type`: 推理策略类型
- `--inference_rate`: 推理频率
- `--device`: 推理设备
- `--ckpt_path`: 模型路径
- `--record_inference`: 是否记录推理数据
- `--inference_timeout`: 推理超时时间

### 3. UI界面扩展

在主界面中添加了"模型推理"标签页，包含：

- 模型设置面板：配置模型类型、设备和路径
- 推理控制面板：开始/停止推理、设置参数
- 显示面板：实时显示相机图像和推理状态

### 4. 创建启动脚本

提供了专用启动脚本 `run_inference.py`，便于用户快速启动系统：

```bash
python scripts/run_inference.py --ckpt_path=/path/to/model
```

## 使用方法

### 1. 准备工作

确保已安装所需依赖：

```bash
pip install -r requirements.txt
```

### 2. 启动系统

使用专用脚本启动：

```bash
python scripts/run_inference.py --ckpt_path=/path/to/model
```

或直接使用主程序启动：

```bash
ros_data_recorder_gui --enable_inference --policy_type=act --device=cuda --ckpt_path=/path/to/model
```

### 3. 加载模型

1. 在"模型推理"标签页中，选择策略类型和设备
2. 点击"浏览..."按钮选择模型检查点文件，或直接输入路径
3. 点击"加载模型"按钮加载模型

### 4. 执行推理

- 点击"开始推理"按钮开始连续推理
- 点击"单步推理"按钮执行一次推理
- 点击"停止推理"按钮结束推理

如果勾选"同时录制推理数据"，系统将在推理过程中同时录制数据。

## 注意事项

1. 确保模型检查点路径正确
2. 确保ROS话题可用
3. 高频率推理可能会对系统性能有影响
4. 推理过程中录制数据可能会增加系统负载

## 故障排除

1. **模型加载失败**
   - 检查模型路径是否正确
   - 确保已安装 lerobot 库
   - 检查设备是否支持（如CUDA）

2. **推理执行失败**
   - 检查ROS话题是否可用
   - 确保机器人连接正常
   - 查看日志获取详细错误信息

3. **系统性能问题**
   - 降低推理频率
   - 使用更高性能的硬件
   - 关闭不必要的功能

## 未来改进

1. 支持更多模型类型
2. 优化推理性能
3. 提供模型训练功能
4. 增加动作可视化功能 