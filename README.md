# 青龙ROS数据记录与处理系统

## 简介

青龙ROS数据记录与处理系统是一个集成的ROS数据采集、处理与可视化工具。通过图形界面，可以方便地记录ROS话题数据、处理数据以及回放机器人轨迹。现已新增**模型推理**功能，可加载预训练模型实时控制机器人。

## 主要功能

1. **实时监控与录制**：记录机器人状态、图像等ROS话题数据
2. **数据管理**：管理录制的数据，支持ZIP压缩与删除
3. **数据处理**：对采集的数据进行同步、转换与重组
4. **轨迹回放**：回放处理后的轨迹数据
5. **模型推理**：加载预训练模型，实时控制机器人

## 安装

### 依赖项

- ROS (Kinetic/Melodic/Noetic)
- Python 3.6+
- Qt5
- PyTorch 1.9+
- OpenCV 4.5+

### 安装方法

```bash
# 克隆仓库
git clone https://github.com/your-organization/ros_data_recorder.git

# 安装依赖
cd ros_data_recorder
pip install -r requirements.txt

# 安装包
pip install -e .
```

## 使用方法

### 启动GUI

```bash
# 直接使用命令行启动
ros_data_recorder_gui

# 或者指定参数
ros_data_recorder_gui --save_dir=/path/to/save --auto_zip
```

### 启动带推理功能的GUI

```bash
# 使用专用脚本启动
python scripts/run_inference.py

# 或者直接添加推理参数
ros_data_recorder_gui --enable_inference --ckpt_path=/path/to/model

# 自定义推理参数
ros_data_recorder_gui --enable_inference --policy_type=act --device=cuda --inference_rate=50.0 --ckpt_path=/path/to/model
```

### 参数说明

#### 基础参数

- `--save_dir`: 数据保存目录
- `--zip_dir`: ZIP文件保存目录
- `--frame_rate`: 状态刷新帧率
- `--compress`: 是否使用压缩图像话题
- `--auto_zip`: 是否自动创建ZIP归档
- `--delete_after_zip`: 创建ZIP后是否删除原始数据
- `--zip_compression`: ZIP压缩级别(0-9)
- `--ignore_missing_topics`: 忽略缺失的话题

#### 推理参数

- `--enable_inference`: 启用模型推理功能
- `--policy_type`: 推理策略类型，可选值: `act`, `diffusion`, `pi0`
- `--inference_rate`: 推理频率，单位Hz
- `--device`: 推理设备类型，可选值: `cuda`, `cpu`, `mps`
- `--ckpt_path`: 模型检查点路径
- `--record_inference`: 是否同时录制推理过程中的数据
- `--inference_timeout`: 推理超时时间(秒)，0表示不限制

## 界面说明

系统提供了六个主要标签页:

1. **实时监控与录制**: 监控话题状态并录制数据
2. **数据管理**: 管理录制的数据
3. **数据处理**: 对数据进行同步处理
4. **轨迹回放**: 回放处理后的轨迹
5. **模型推理**: 加载预训练模型并执行推理控制
6. **设置**: 配置系统参数

## 模型推理功能说明

模型推理标签页提供以下功能：

1. **模型设置**
   - 选择策略类型（ACT、扩散策略、PI0）
   - 选择推理设备（CUDA、CPU、MPS）
   - 设置模型检查点路径
   - 加载模型

2. **推理控制**
   - 设置推理频率
   - 设置推理超时时间
   - 同时录制选项
   - 开始/停止推理
   - 单步推理执行

3. **实时显示**
   - 显示前方、左侧、右侧相机图像
   - 显示推理状态和推理时间

## License

MIT License

## 开发者

青龙团队 