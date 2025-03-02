# 青龙ROS数据记录与处理系统

## 简介

青龙ROS数据记录与处理系统是一个集成的ROS数据采集、处理与可视化工具。通过图形界面，可以方便地记录ROS话题数据、处理数据以及回放机器人轨迹。

## 主要功能

1. **实时监控与录制**：记录机器人状态、图像等ROS话题数据
2. **数据管理**：管理录制的数据，支持ZIP压缩与删除
3. **数据处理**：对采集的数据进行同步、转换与重组
4. **轨迹回放**：回放处理后的轨迹数据

## 安装

### 依赖项

- ROS (Kinetic/Melodic/Noetic)
- Python 3.6+
- Qt5

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

### 参数说明

- `--save_dir`: 数据保存目录
- `--zip_dir`: ZIP文件保存目录
- `--frame_rate`: 状态刷新帧率
- `--compress`: 是否使用压缩图像话题
- `--auto_zip`: 是否自动创建ZIP归档
- `--delete_after_zip`: 创建ZIP后是否删除原始数据
- `--zip_compression`: ZIP压缩级别(0-9)
- `--ignore_missing_topics`: 忽略缺失的话题

## 界面说明

系统提供了五个主要标签页:

1. **实时监控与录制**: 监控话题状态并录制数据
2. **数据管理**: 管理录制的数据
3. **数据处理**: 对数据进行同步处理
4. **轨迹回放**: 回放处理后的轨迹
5. **设置**: 配置系统参数

## License

MIT License

## 开发者

青龙团队 