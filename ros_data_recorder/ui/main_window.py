#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
主窗口模块，包含应用程序的主界面
"""

import os
import sys
import time
import rospy
from datetime import datetime
from PyQt5.QtWidgets import (QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, 
                          QPushButton, QLabel, QTableWidget, QTableWidgetItem, QGroupBox, 
                          QProgressBar, QTextEdit, QTabWidget, QFileDialog, QGridLayout,
                          QSpacerItem, QSizePolicy, QCheckBox, QComboBox, QScrollArea,
                          QFrame, QHeaderView, QSplitter, QMessageBox, QListWidget,
                          QListWidgetItem, QDialog, QRadioButton, QButtonGroup, QLineEdit, QApplication, QGridLayout, QDoubleSpinBox, QSpinBox)
from PyQt5.QtCore import QTimer, Qt, pyqtSignal, pyqtSlot, QThread
from PyQt5.QtGui import QColor, QFont, QPixmap, QIcon, QFontDatabase, QTextCursor, QFont, QPainter, QImage
import shutil
import threading
import numpy as np
import cv2
import torch

# 导入自定义模块
from ..core.topic_monitor import TopicMonitor
from ..core.data_storage import DataStorage
from .dialogs import TrajectoryPlayerDialog, ProcessResultDialog
from ..workers.zip_worker import ZipWorker
from ..workers.process_worker import DataProcessWorker

try:
    from sensor_msgs.msg import Image, JointState, CompressedImage
    from cv_bridge import CvBridge
    from mocap2robot.msg import vp_control
    from sdk.msg import PuppetState, AlohaCmd
except ImportError as e:
    print(f"警告: 部分ROS依赖导入失败 ({e})，将在运行时处理")

class ROSRecorderGUI(QMainWindow):
    """ROS数据记录器与处理系统GUI主窗口"""
    
    def __init__(self, args):
        super().__init__()
        
        # 设置参数
        self.args = args
        # 设置默认ZIP目录
        if not hasattr(self.args, 'zip_dir') or self.args.zip_dir is None:
            self.args.zip_dir = self.args.save_dir  # 默认与save_dir相同
        
        # 初始化数据
        self.topic_monitor = TopicMonitor()
        self.data_storage = DataStorage(args.save_dir, args=args)
        self.is_recording = False
        self.is_compressing = False  # 是否在压缩中标志
        self.is_processing = False   # 是否在处理数据中标志
        self.recording_start_time = 0
        
        # 数据缓存
        self.latest_gripper_action = [0.0, 0.0]  # 最新的夹爪控制指令
        self.latest_grippers = [0.0, 0.0]  # 最新的夹爪实际位置
        
        # 设置界面
        self.setup_chinese_ui()
        self.init_ui()
        
        # 设置定时器，更新界面状态
        self.update_timer = QTimer(self)
        self.update_timer.timeout.connect(self.update_status)
        self.update_timer.start(500)  # 每0.5秒更新一次界面
        
        # 初始化ROS节点
        self.init_ros_node()
        
        # 初始化推理引擎
        if self.args.enable_inference:
            try:
                from ..core.inference_engine import InferenceEngine
                self.inference_engine = InferenceEngine(args=self.args)
                self.log_message("推理引擎初始化成功")
            except ImportError as e:
                self.log_message(f"推理引擎初始化失败: {str(e)}")
                self.args.enable_inference = False
        else:
            self.inference_engine = None
            
        # 初始化工作线程
        self.init_workers()
        
        # 显示就绪
        self.log_message("ROS数据记录器已初始化，等待话题数据...")
    
    def setup_chinese_ui(self):
        """设置中文UI支持"""
        # 尝试设置中文字体
        chinese_fonts = ['Microsoft YaHei', 'SimHei', 'WenQuanYi Micro Hei', 'Noto Sans CJK SC', 'Noto Sans SC']
        
        app = QApplication.instance()
        font_db = QFontDatabase()
        system_fonts = font_db.families()
        
        # 查找第一个可用的中文字体
        available_font = None
        for font_name in chinese_fonts:
            if any(font_name.lower() in f.lower() for f in system_fonts):
                available_font = font_name
                break
        
        # 如果找到支持中文的字体，设置为应用默认字体
        if available_font:
            font = QFont(available_font, 9)
            app.setFont(font)
            print(f"已设置中文字体: {available_font}")
        else:
            # 无合适中文字体，使用系统默认
            print("未找到合适的中文字体，使用系统默认字体")
    
    def init_ui(self):
        """初始化用户界面"""
        # 设置窗口
        self.setWindowTitle("青龙ROS1数据记录与处理系统")
        self.setGeometry(100, 100, 1100, 700)
        
        # 创建主窗口部件
        main_widget = QWidget()
        self.setCentralWidget(main_widget)
        
        # 主布局
        main_layout = QVBoxLayout(main_widget)
        
        # 创建标题标签
        title_label = QLabel("青龙ROS1数据记录与处理系统")
        title_label.setFont(QFont('Arial', 18, QFont.Bold))
        title_label.setAlignment(Qt.AlignCenter)
        main_layout.addWidget(title_label)
        
        # 创建标签页
        self.tabs = QTabWidget()
        main_layout.addWidget(self.tabs)
        
        # 创建各个标签页
        record_tab = QWidget()
        manage_tab = QWidget()
        process_tab = QWidget()
        settings_tab = QWidget()
        playback_tab = QWidget()
        inference_tab = QWidget()
        
        # 添加标签页到标签控件
        self.tabs.addTab(record_tab, "实时监控与录制")
        self.tabs.addTab(manage_tab, "数据管理")
        self.tabs.addTab(process_tab, "数据处理") 
        self.tabs.addTab(playback_tab, "轨迹回放")
        self.tabs.addTab(inference_tab, "模型推理")
        self.tabs.addTab(settings_tab, "设置")
        
        # 初始化各标签页的布局
        self.setup_record_tab(record_tab)
        self.setup_manage_tab(manage_tab)
        self.setup_process_tab(process_tab)
        self.setup_settings_tab(settings_tab)
        self.setup_playback_tab(playback_tab)
        
        # 初始化推理标签页
        self.setup_inference_tab(inference_tab)
        
        # 状态栏
        self.statusbar = self.statusBar()
        self.statusbar.showMessage("就绪")
    
    def setup_record_tab(self, tab):
        """设置录制标签页的UI"""
        layout = QVBoxLayout(tab)
        
        # 话题监控区
        topic_group = QGroupBox("话题状态监控")
        topic_layout = QVBoxLayout()
        
        # 话题表格
        self.topic_table = QTableWidget(7, 4)  # 7行4列
        self.topic_table.setHorizontalHeaderLabels(["话题", "频率", "消息计数", "状态"])
        self.topic_table.verticalHeader().setVisible(False)
        self.topic_table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        
        # 填充话题表格
        topic_names = ['front_camera', 'left_camera', 'right_camera', 
                       'puppet_state', 'gripper_left', 'gripper_right', 'gripper_action']
        for i, topic in enumerate(topic_names):
            self.topic_table.setItem(i, 0, QTableWidgetItem(topic))
            self.topic_table.setItem(i, 1, QTableWidgetItem("0.0 Hz"))
            self.topic_table.setItem(i, 2, QTableWidgetItem("0"))
            status_item = QTableWidgetItem("WAITING")
            status_item.setForeground(QColor("orange"))
            self.topic_table.setItem(i, 3, status_item)
            
        topic_layout.addWidget(self.topic_table)
        topic_group.setLayout(topic_layout)
        layout.addWidget(topic_group)
        
        # 录制状态区
        recording_group = QGroupBox("录制状态")
        recording_layout = QVBoxLayout()
        
        # 录制状态信息
        status_layout = QGridLayout()
        
        # 当前状态
        status_layout.addWidget(QLabel("当前状态:"), 0, 0)
        self.record_status_label = QLabel("未录制")
        self.record_status_label.setStyleSheet("font-weight: bold; color: gray;")
        status_layout.addWidget(self.record_status_label, 0, 1)
        
        # 录制时长
        status_layout.addWidget(QLabel("录制时长:"), 0, 2)
        self.record_duration_label = QLabel("00:00:00")
        status_layout.addWidget(self.record_duration_label, 0, 3)
        
        # 状态数据帧数
        status_layout.addWidget(QLabel("状态帧数:"), 1, 0)
        self.state_frames_label = QLabel("0")
        status_layout.addWidget(self.state_frames_label, 1, 1)
        
        # 图像总数
        status_layout.addWidget(QLabel("图像总数:"), 1, 2)
        self.image_count_label = QLabel("0")
        status_layout.addWidget(self.image_count_label, 1, 3)
        
        # 保存目录
        status_layout.addWidget(QLabel("保存目录:"), 2, 0)
        self.save_dir_label = QLabel("-")
        self.save_dir_label.setWordWrap(True)
        status_layout.addWidget(self.save_dir_label, 2, 1, 1, 3)
        
        recording_layout.addLayout(status_layout)
        
        # ZIP压缩进度条 - 移到第一页
        progress_layout = QHBoxLayout()
        progress_layout.addWidget(QLabel("ZIP压缩进度:"))
        
        self.zip_progress = QProgressBar()
        self.zip_progress.setRange(0, 100)
        self.zip_progress.setValue(0)
        self.zip_progress.setVisible(False)
        progress_layout.addWidget(self.zip_progress)
        
        recording_layout.addLayout(progress_layout)
        
        # 控制按钮区
        buttons_layout = QHBoxLayout()
        
        # 开始录制按钮
        self.start_button = QPushButton("开始录制")
        self.start_button.setStyleSheet("background-color: #4CAF50; color: white;")
        self.start_button.clicked.connect(self.start_recording)
        buttons_layout.addWidget(self.start_button)
        
        # 停止录制按钮
        self.stop_button = QPushButton("停止录制")
        self.stop_button.setStyleSheet("background-color: #F44336; color: white;")
        self.stop_button.clicked.connect(self.stop_recording)
        self.stop_button.setEnabled(False)
        buttons_layout.addWidget(self.stop_button)
        
        # 重置按钮
        self.reset_button = QPushButton("重置")
        self.reset_button.clicked.connect(self.reset_state)
        buttons_layout.addWidget(self.reset_button)
        
        recording_layout.addLayout(buttons_layout)
        recording_group.setLayout(recording_layout)
        layout.addWidget(recording_group)
        
        # 日志区
        log_group = QGroupBox("日志信息")
        log_layout = QVBoxLayout()
        
        self.log_text = QTextEdit()
        self.log_text.setReadOnly(True)
        log_layout.addWidget(self.log_text)
        
        log_group.setLayout(log_layout)
        layout.addWidget(log_group)
    
    def setup_manage_tab(self, tab):
        """设置数据管理标签页的UI"""
        layout = QVBoxLayout(tab)
        
        # 录制列表区域
        recordings_group = QGroupBox("录制数据列表")
        recordings_layout = QVBoxLayout()
        
        # 录制列表表格
        self.recordings_table = QTableWidget(0, 3)  # 0行3列
        self.recordings_table.setHorizontalHeaderLabels(["名称", "类型", "信息"])
        self.recordings_table.verticalHeader().setVisible(False)
        self.recordings_table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self.recordings_table.setSelectionBehavior(QTableWidget.SelectRows)
        self.recordings_table.setSelectionMode(QTableWidget.SingleSelection)
        
        recordings_layout.addWidget(self.recordings_table)
        
        # 操作按钮区
        buttons_layout = QHBoxLayout()
        
        # 刷新按钮
        refresh_button = QPushButton("刷新列表")
        refresh_button.clicked.connect(self.refresh_recordings_list)
        buttons_layout.addWidget(refresh_button)
        
        # 打开目录按钮
        open_dir_button = QPushButton("打开目录")
        open_dir_button.clicked.connect(self.open_selected_dir)
        buttons_layout.addWidget(open_dir_button)
        
        # 创建ZIP按钮
        self.create_zip_button = QPushButton("创建ZIP")
        self.create_zip_button.clicked.connect(self.create_zip_for_selected)
        buttons_layout.addWidget(self.create_zip_button)
        
        # 删除按钮
        delete_button = QPushButton("删除")
        delete_button.setStyleSheet("background-color: #F44336; color: white;")
        delete_button.clicked.connect(self.delete_selected)
        buttons_layout.addWidget(delete_button)
        
        recordings_layout.addLayout(buttons_layout)
        
        recordings_group.setLayout(recordings_layout)
        layout.addWidget(recordings_group)
        
        # 初始加载录制列表
        self.refresh_recordings_list()
    
    def setup_process_tab(self, tab):
        """设置数据处理标签页的UI"""
        layout = QVBoxLayout(tab)
        
        # 数据处理区域
        process_group = QGroupBox("数据处理")
        process_layout = QVBoxLayout()
        
        # 源数据目录选择
        source_layout = QHBoxLayout()
        source_layout.addWidget(QLabel("源数据目录:"))
        self.source_dir_edit = QLabel(self.args.save_dir)
        source_layout.addWidget(self.source_dir_edit)
        
        source_browse_button = QPushButton("浏览...")
        source_browse_button.clicked.connect(self.browse_source_dir)
        source_layout.addWidget(source_browse_button)
        
        process_layout.addLayout(source_layout)
        
        # 输出目录选择
        output_layout = QHBoxLayout()
        output_layout.addWidget(QLabel("输出目录:"))
        self.output_dir_edit = QLabel(os.path.join(self.args.save_dir, "processed"))
        output_layout.addWidget(self.output_dir_edit)
        
        output_browse_button = QPushButton("浏览...")
        output_browse_button.clicked.connect(self.browse_output_dir)
        output_layout.addWidget(output_browse_button)
        
        process_layout.addLayout(output_layout)
        
        # 处理选项
        options_layout = QGridLayout()
        
        # 处理线程数
        options_layout.addWidget(QLabel("处理线程数:"), 0, 0)
        self.process_threads_combo = QComboBox()
        for i in range(1, 9):
            self.process_threads_combo.addItem(f"{i}")
        self.process_threads_combo.setCurrentIndex(2)  # 默认3线程
        options_layout.addWidget(self.process_threads_combo, 0, 1)
        
        # 自动删除失败结果选项
        options_layout.addWidget(QLabel("自动删除失败结果:"), 1, 0)
        self.delete_failed_check = QCheckBox()
        self.delete_failed_check.setChecked(True)  # 默认删除失败的处理
        options_layout.addWidget(self.delete_failed_check, 1, 1)
        
        process_layout.addLayout(options_layout)
        
        # 处理按钮
        start_process_button = QPushButton("开始处理")
        start_process_button.setStyleSheet("background-color: #4CAF50; color: white;")
        start_process_button.clicked.connect(self.start_data_processing)
        process_layout.addWidget(start_process_button)
        
        # 处理进度
        self.process_log = QTextEdit()
        self.process_log.setReadOnly(True)
        self.process_log.setMinimumHeight(300)
        process_layout.addWidget(self.process_log)
        
        process_group.setLayout(process_layout)
        layout.addWidget(process_group)
    
    def setup_settings_tab(self, tab):
        """设置配置标签页的UI"""
        layout = QVBoxLayout(tab)
        
        # 存储设置区
        storage_group = QGroupBox("存储设置")
        storage_layout = QGridLayout()
        
        # 保存目录
        storage_layout.addWidget(QLabel("保存目录:"), 0, 0)
        self.save_dir_edit = QLabel(self.args.save_dir)
        storage_layout.addWidget(self.save_dir_edit, 0, 1)
        
        browse_button = QPushButton("浏览...")
        browse_button.clicked.connect(self.browse_save_dir)
        storage_layout.addWidget(browse_button, 0, 2)
        
        # ZIP保存目录
        storage_layout.addWidget(QLabel("ZIP保存目录:"), 1, 0)
        self.zip_dir_edit = QLabel(self.args.zip_dir)
        storage_layout.addWidget(self.zip_dir_edit, 1, 1)
        
        zip_browse_button = QPushButton("浏览...")
        zip_browse_button.clicked.connect(self.browse_zip_dir)
        storage_layout.addWidget(zip_browse_button, 1, 2)
        
        # ZIP设置
        storage_layout.addWidget(QLabel("自动创建ZIP:"), 2, 0)
        self.auto_zip_check = QCheckBox()
        self.auto_zip_check.setChecked(self.args.auto_zip)
        storage_layout.addWidget(self.auto_zip_check, 2, 1)
        
        storage_layout.addWidget(QLabel("ZIP后删除原始文件:"), 3, 0)
        self.delete_after_zip_check = QCheckBox()
        self.delete_after_zip_check.setChecked(self.args.delete_after_zip)
        storage_layout.addWidget(self.delete_after_zip_check, 3, 1)
        
        storage_layout.addWidget(QLabel("ZIP压缩级别:"), 4, 0)
        self.zip_level_combo = QComboBox()
        for i in range(10):
            self.zip_level_combo.addItem(f"{i} - {'无压缩' if i==0 else '最大压缩' if i==9 else '标准压缩'}")
        self.zip_level_combo.setCurrentIndex(self.args.zip_compression)
        storage_layout.addWidget(self.zip_level_combo, 4, 1)
        
        storage_group.setLayout(storage_layout)
        layout.addWidget(storage_group)
        
        # 话题设置区
        topic_group = QGroupBox("话题设置")
        topic_layout = QGridLayout()
        
        # 是否使用压缩图像
        topic_layout.addWidget(QLabel("使用压缩图像:"), 0, 0)
        self.compress_check = QCheckBox()
        self.compress_check.setChecked(self.args.compress)
        topic_layout.addWidget(self.compress_check, 0, 1)
        
        # 忽略缺失话题
        topic_layout.addWidget(QLabel("忽略缺失的话题:"), 1, 0)
        self.ignore_missing_topics_check = QCheckBox()
        self.ignore_missing_topics_check.setChecked(self.args.ignore_missing_topics)
        topic_layout.addWidget(self.ignore_missing_topics_check, 1, 1)
        
        topic_group.setLayout(topic_layout)
        layout.addWidget(topic_group)
        
        # 保存设置按钮
        save_button = QPushButton("保存设置")
        save_button.clicked.connect(self.save_settings)
        layout.addWidget(save_button)
        
        # 添加弹性空间
        layout.addStretch()
    
    def setup_playback_tab(self, tab):
        """设置回放标签页的UI"""
        layout = QVBoxLayout(tab)
        
        # 轨迹文件选择区
        file_group = QGroupBox("轨迹文件选择")
        file_layout = QVBoxLayout(file_group)
        
        # 当前目录的处理后结果
        results_layout = QHBoxLayout()
        results_layout.addWidget(QLabel("当前处理结果目录:"))
        
        self.playback_dir_combo = QComboBox()
        results_layout.addWidget(self.playback_dir_combo)
        
        refresh_button = QPushButton("刷新")
        refresh_button.clicked.connect(self.refresh_playback_dirs)
        results_layout.addWidget(refresh_button)
        
        file_layout.addLayout(results_layout)
        
        # 手动选择HDF5文件
        hdf5_layout = QHBoxLayout()
        hdf5_layout.addWidget(QLabel("或直接选择HDF5文件:"))
        
        self.hdf5_path_edit = QLineEdit()
        self.hdf5_path_edit.setReadOnly(True)
        hdf5_layout.addWidget(self.hdf5_path_edit)
        
        hdf5_browse_button = QPushButton("浏览...")
        hdf5_browse_button.clicked.connect(self.browse_hdf5_file)
        hdf5_layout.addWidget(hdf5_browse_button)
        
        file_layout.addLayout(hdf5_layout)
        
        layout.addWidget(file_group)
        
        # 启动回放按钮
        start_playback_button = QPushButton("启动轨迹回放")
        start_playback_button.setStyleSheet("background-color: #4CAF50; color: white; font-size: 14px; padding: 10px;")
        start_playback_button.clicked.connect(self.start_trajectory_playback)
        layout.addWidget(start_playback_button)
        
        # 说明文本
        info_text = QTextEdit()
        info_text.setReadOnly(True)
        info_text.setPlainText("""
    使用说明:

    1. 轨迹回放需要已经处理后的数据，包含HDF5文件和同步后的图像。
    2. 您可以选择下拉列表中的处理结果目录，或直接浏览选择HDF5文件。
    3. 点击"启动轨迹回放"按钮打开回放控制窗口。
    4. 在回放控制窗口中，您可以:
    - 控制回放的暂停和继续
    - 调整回放速率
    - 选择是否显示相机图像
        
    注意事项:
    - 回放过程中会发布机器人控制指令到ROS话题，确保安全。
    - 轨迹回放时请保持对机器人的监控，必要时随时停止。
    - 如果显示图像，请确保数据目录中包含camera文件夹及对应帧图像。
        """)
        layout.addWidget(info_text)
        
        # 所有UI元素创建完成后再刷新目录列表
        self.refresh_playback_dirs()
    
    def setup_inference_tab(self, inference_tab):
        """初始化推理标签页"""
        # 创建推理标签页布局
        inference_layout = QVBoxLayout(inference_tab)
        
        # 模型设置区域
        model_group = QGroupBox("模型设置")
        model_layout = QGridLayout()
        
        # 策略类型选择
        self.policy_type_label = QLabel("策略类型:")
        self.policy_type_combo = QComboBox()
        self.policy_type_combo.addItems(["ACT", "扩散策略", "PI0"])
        model_layout.addWidget(self.policy_type_label, 0, 0)
        model_layout.addWidget(self.policy_type_combo, 0, 1)
        
        # 设备选择
        self.device_label = QLabel("推理设备:")
        self.device_combo = QComboBox()
        self.device_combo.addItems(["CUDA", "CPU", "MPS"])
        if not torch.cuda.is_available():
            # 如果不支持CUDA，设置为CPU
            self.device_combo.setCurrentIndex(1)
        model_layout.addWidget(self.device_label, 0, 2)
        model_layout.addWidget(self.device_combo, 0, 3)
        
        # 检查点路径
        self.ckpt_path_label = QLabel("模型路径:")
        self.ckpt_path_edit = QLineEdit()
        self.ckpt_path_edit.setPlaceholderText("输入模型文件夹路径")
        self.ckpt_path_button = QPushButton("浏览...")
        self.ckpt_path_button.clicked.connect(self.browse_ckpt_path)
        model_layout.addWidget(self.ckpt_path_label, 1, 0)
        model_layout.addWidget(self.ckpt_path_edit, 1, 1, 1, 2)
        model_layout.addWidget(self.ckpt_path_button, 1, 3)
        
        # 加载模型按钮
        self.load_model_button = QPushButton("加载模型")
        self.load_model_button.clicked.connect(self.load_model)
        model_layout.addWidget(self.load_model_button, 2, 0, 1, 4)
        
        # 设置模型组布局
        model_group.setLayout(model_layout)
        inference_layout.addWidget(model_group)
        
        # 推理控制区域
        control_group = QGroupBox("推理控制")
        control_layout = QGridLayout()
        
        # 推理频率设置
        self.inference_rate_label = QLabel("推理频率(Hz):")
        self.inference_rate_spin = QDoubleSpinBox()
        self.inference_rate_spin.setRange(1.0, 200.0)
        self.inference_rate_spin.setValue(50.0)
        self.inference_rate_spin.setSingleStep(1.0)
        control_layout.addWidget(self.inference_rate_label, 0, 0)
        control_layout.addWidget(self.inference_rate_spin, 0, 1)
        
        # 超时设置
        self.timeout_label = QLabel("超时时间(秒):")
        self.timeout_spin = QSpinBox()
        self.timeout_spin.setRange(0, 3600)
        self.timeout_spin.setValue(0)
        self.timeout_spin.setSpecialValueText("无限制")
        control_layout.addWidget(self.timeout_label, 0, 2)
        control_layout.addWidget(self.timeout_spin, 0, 3)
        
        # 同时录制选项
        self.record_inference_check = QCheckBox("同时录制推理数据")
        control_layout.addWidget(self.record_inference_check, 1, 0, 1, 2)
        
        # 推理控制按钮
        self.start_inference_button = QPushButton("开始推理")
        self.start_inference_button.clicked.connect(self.start_inference)
        self.start_inference_button.setEnabled(False)  # 初始状态下禁用
        
        self.stop_inference_button = QPushButton("停止推理")
        self.stop_inference_button.clicked.connect(self.stop_inference)
        self.stop_inference_button.setEnabled(False)  # 初始状态下禁用
        
        self.single_step_button = QPushButton("单步推理")
        self.single_step_button.clicked.connect(self.single_step_inference)
        self.single_step_button.setEnabled(False)  # 初始状态下禁用
        
        control_layout.addWidget(self.start_inference_button, 2, 0, 1, 2)
        control_layout.addWidget(self.stop_inference_button, 2, 2, 1, 1)
        control_layout.addWidget(self.single_step_button, 2, 3, 1, 1)
        
        # 设置控制组布局
        control_group.setLayout(control_layout)
        inference_layout.addWidget(control_group)
        
        # 状态显示区域
        status_group = QGroupBox("运行状态")
        status_layout = QVBoxLayout()
        
        # 状态显示
        self.inference_info_label = QLabel("推理状态: 未初始化")
        self.inference_time_label = QLabel("推理时间: -- ms")
        status_layout.addWidget(self.inference_info_label)
        status_layout.addWidget(self.inference_time_label)
        
        # 设置状态组布局
        status_group.setLayout(status_layout)
        inference_layout.addWidget(status_group)
        
        # 初始状态设置
        if not self.args.enable_inference:
            inference_tab.setEnabled(False)
            info_label = QLabel("推理功能未启用，请使用 --enable_inference 启动参数")
            info_label.setAlignment(Qt.AlignCenter)
            inference_layout.addWidget(info_label)
    
    def init_ros_node(self):
        """初始化ROS节点和订阅器"""
        try:
            rospy.init_node('data_recorder_gui', anonymous=True)
            
            # 设置图像订阅器
            rospy.Subscriber(
                self.args.img_front_topic,
                CompressedImage if self.args.compress else Image,
                self.img_front_callback,
                queue_size=1000,
                tcp_nodelay=True
            )
            rospy.Subscriber(
                self.args.img_left_topic,
                CompressedImage if self.args.compress else Image,
                self.img_left_callback,
                queue_size=1000,
                tcp_nodelay=True
            )
            rospy.Subscriber(
                self.args.img_right_topic,
                CompressedImage if self.args.compress else Image,
                self.img_right_callback,
                queue_size=1000,
                tcp_nodelay=True
            )
            
            # 设置其他数据订阅器
            rospy.Subscriber(
                self.args.puppet_state_topic,
                PuppetState,
                self.puppet_state_callback,
                queue_size=1000,
                tcp_nodelay=True
            )
            rospy.Subscriber(
                self.args.gripper_left_topic,
                JointState,
                self.gripper_left_callback,
                queue_size=1000,
                tcp_nodelay=True
            )
            rospy.Subscriber(
                self.args.gripper_right_topic,
                JointState,
                self.gripper_right_callback,
                queue_size=1000,
                tcp_nodelay=True
            )
            rospy.Subscriber(
                self.args.gripper_action_topic,
                JointState,
                self.gripper_action_callback,
                queue_size=1000,
                tcp_nodelay=True
            )
            
            self.log_message("ROS节点初始化成功")
        except Exception as e:
            self.log_message(f"ROS节点初始化失败: {str(e)}", error=True)
    
    # ROS话题回调函数
    def img_front_callback(self, msg):
        """前方相机图像回调"""
        self.topic_monitor.update_topic('front_camera')
        if self.is_recording:
            self.data_storage.save_image_async(msg, 'front', self.args.compress)

    def img_left_callback(self, msg):
        """左侧相机图像回调"""
        self.topic_monitor.update_topic('left_camera')
        if self.is_recording:
            self.data_storage.save_image_async(msg, 'left', self.args.compress)

    def img_right_callback(self, msg):
        """右侧相机图像回调"""
        self.topic_monitor.update_topic('right_camera')
        if self.is_recording:
            self.data_storage.save_image_async(msg, 'right', self.args.compress)

    def puppet_state_callback(self, msg):
        """机器人状态回调"""
        self.topic_monitor.update_topic('puppet_state')
        if self.is_recording:
            timestamp = msg.header.stamp.to_sec()
            
            # 添加状态数据
            self.data_storage.add_state_data(
                timestamp, 
                msg.arm_left.position, 
                msg.arm_right.position,
                self.latest_grippers
            )
            
            # 添加动作数据
            self.data_storage.add_action_data(
                timestamp,
                msg.arm_left_exp.position,
                msg.arm_right_exp.position,
                self.latest_gripper_action
            )

    def gripper_left_callback(self, msg):
        """左夹爪回调"""
        self.topic_monitor.update_topic('gripper_left')
        if len(msg.position) > 0:
            self.latest_grippers[0] = msg.position[0]

    def gripper_right_callback(self, msg):
        """右夹爪回调"""
        self.topic_monitor.update_topic('gripper_right')
        if len(msg.position) > 0:
            self.latest_grippers[1] = msg.position[0]

    def gripper_action_callback(self, msg):
        """夹爪控制指令回调"""
        self.topic_monitor.update_topic('gripper_action')
        if len(msg.position) > 0:
            self.latest_gripper_action = list(msg.position)
    
    def update_status(self):
        """更新界面状态"""
        # 更新话题状态表格
        for i, (topic_name, topic) in enumerate(self.topic_monitor.topics.items()):
            # 更新频率
            self.topic_table.item(i, 1).setText(f"{topic.freq:.1f} Hz")
            # 更新消息计数
            self.topic_table.item(i, 2).setText(f"{topic.count}")
            # 更新状态
            status = topic.get_status(self.topic_monitor.timeout)
            status_item = self.topic_table.item(i, 3)
            status_item.setText(status)
            
            if status == 'WAITING':
                status_item.setForeground(QColor("orange"))
            elif status == 'TIMEOUT':
                status_item.setForeground(QColor("red"))
            else:
                status_item.setForeground(QColor("green"))
        
        # 更新录制状态信息
        if self.is_recording:
            # 计算当前录制时长
            current_duration = time.time() - self.recording_start_time
            self.data_storage.recording_duration = current_duration
            
            # 更新录制状态标签
            self.record_status_label.setText("正在录制")
            self.record_status_label.setStyleSheet("font-weight: bold; color: green;")
            
            # 更新时长
            duration_str = self.data_storage.format_duration(current_duration)
            self.record_duration_label.setText(duration_str)
            
            # 更新帧数
            self.state_frames_label.setText(f"{len(self.data_storage.states)}")
            
            # 计算图像总数
            image_count = sum(topic.count for name, topic in self.topic_monitor.topics.items() 
                            if 'camera' in name)
            self.image_count_label.setText(f"{image_count}")
            
        else:
            # 未在录制状态
            if self.is_compressing:
                self.record_status_label.setText("ZIP压缩中")
                self.record_status_label.setStyleSheet("font-weight: bold; color: blue;")
            else:
                self.record_status_label.setText("未录制")
                self.record_status_label.setStyleSheet("font-weight: bold; color: gray;")
        
        # 更新按钮状态
        self.update_button_states()
    
    def update_button_states(self):
        """更新按钮状态"""
        # 如果在压缩中，禁用开始录制
        if self.is_compressing or self.is_processing:
            self.start_button.setEnabled(False)
            # 在管理页面也禁用ZIP按钮
            self.create_zip_button.setEnabled(False)
        else:
            # 不在压缩中，根据是否在录制决定
            self.start_button.setEnabled(not self.is_recording)
            self.create_zip_button.setEnabled(True)
            
        # 停止按钮仅在录制时启用
        self.stop_button.setEnabled(self.is_recording)
    
    def log_message(self, message, error=False):
        """添加日志消息"""
        timestamp = datetime.now().strftime('%H:%M:%S')
        if error:
            log_entry = f"<span style='color:red'>[{timestamp}] ERROR: {message}</span>"
        else:
            log_entry = f"[{timestamp}] {message}"
        
        self.log_text.append(log_entry)
        # 滚动到底部
        self.log_text.verticalScrollBar().setValue(self.log_text.verticalScrollBar().maximum())
        
        # 同时在控制台输出
        print(f"[{timestamp}] {'ERROR: ' if error else ''}{message}")
    
    # 录制功能
    def start_recording(self):
        """开始录制数据"""
        if self.is_recording:
            self.log_message("已在录制中，请先停止当前录制")
            return
            
        # 检查是否在压缩中
        if self.is_compressing:
            self.log_message("ZIP压缩进行中，请等待压缩完成后再开始录制", error=True)
            return
            
        # 检查所有话题状态
        if not self.topic_monitor.all_topics_active():
            # 如果设置了忽略缺失话题选项，则检查核心话题
            if self.args.ignore_missing_topics:
                # 显示警告但继续录制
                self.log_message("警告：部分非核心话题未就绪，但仍将继续录制", error=True)
            else:
                self.log_message("警告：有话题未就绪，请等待所有话题变为活跃状态", error=True)
                return
            
        # 重置状态
        self.topic_monitor.reset_counts()
        
        # 创建新的录制
        save_dir = self.data_storage.start_new_recording()
        self.is_recording = True
        self.recording_start_time = time.time()
        
        # 更新UI
        self.save_dir_label.setText(save_dir)
        self.update_button_states()
        
        # 记录开始信息
        self.log_message(f"开始新录制，保存到: {save_dir}")
    
    def stop_recording(self):
        """停止录制数据"""
        if not self.is_recording:
            return
            
        self.is_recording = False
        self.log_message("结束录制...")
        
        # 更新录制时长
        self.data_storage.recording_duration = time.time() - self.recording_start_time
        
        # 保存数据
        if len(self.data_storage.states) > 0 or len(self.data_storage.actions) > 0:
            self.data_storage.save_csv_data()
            duration_str = self.data_storage.format_duration(self.data_storage.recording_duration)
            frames = len(self.data_storage.states)
            
            self.log_message(f"数据已保存到: {self.data_storage.save_dir}")
            self.log_message(f"录制总时长: {duration_str}, 总帧数: {frames}")
            
            # 创建ZIP归档
            if self.auto_zip_check.isChecked():
                # 设置压缩状态
                self.is_compressing = True
                self.update_button_states()
                
                # 显示进度条
                self.zip_progress.setValue(0)
                self.zip_progress.setVisible(True)
                
                # 启动异步压缩任务
                self.zip_worker = ZipWorker(
                    self.data_storage,
                    self.data_storage.save_dir,
                    compression_level=self.zip_level_combo.currentIndex(),
                    delete_after=self.delete_after_zip_check.isChecked(),
                    output_dir=self.args.zip_dir  # 传递ZIP输出目录
                )
                self.zip_worker.progress_signal.connect(self.update_zip_progress)
                self.zip_worker.finished_signal.connect(self.on_zip_finished)
                self.zip_worker.error_signal.connect(self.on_zip_error)
                self.zip_worker.start()
                
                self.log_message("正在创建ZIP归档，请稍候...")
        
        # 更新UI
        self.save_dir_label.setText("-")
        self.update_button_states()
        
        # 如果没有自动创建ZIP，则立即重置存储状态
        if not self.auto_zip_check.isChecked():
            # 重置存储状态
            self.data_storage.reset()
            
            # 刷新录制列表
            self.refresh_recordings_list()
    
    def reset_state(self):
        """重置录制状态"""
        if self.is_recording:
            self.stop_recording()
        
        # 如果在压缩中，不允许重置
        if self.is_compressing:
            self.log_message("ZIP压缩进行中，请等待压缩完成后再重置", error=True)
            return
            
        # 重置所有状态
        self.data_storage.reset()
        self.log_message("状态已重置，准备新的录制...")
    
    # 数据管理功能
    def refresh_recordings_list(self):
        """刷新录制列表"""
        # 获取录制摘要
        summary = self.data_storage.get_session_summary()
        
        # 清空表格
        self.recordings_table.setRowCount(0)
        
        # 创建一个字典，将ZIP文件与目录进行匹配
        # 格式: {目录名: {zip文件名, zip信息, source_dir}}
        zip_mapping = {}
        zip_items = []
        dir_items = []
        
        # 分离目录和ZIP文件，建立映射关系
        for item in summary:
            name = item['name']
            if item['type'] == 'zip':
                # 去掉.zip后缀获取原始目录名
                base_name = name[:-4] if name.endswith('.zip') else name
                zip_mapping[base_name] = {
                    'name': name, 
                    'info': item['info'],
                    'source_dir': item['source_dir']
                }
                zip_items.append(item)
            else:
                dir_items.append(item)
        
        # 先添加所有目录条目，如果有对应的ZIP文件则在下方添加
        row_idx = 0
        for dir_item in sorted(dir_items, key=lambda x: x['name']):
            # 添加目录行
            self.recordings_table.insertRow(row_idx)
            dir_name = dir_item['name']
            
            # 设置目录名称
            name_item = QTableWidgetItem(dir_name)
            self.recordings_table.setItem(row_idx, 0, name_item)
            
            # 设置类型
            type_item = QTableWidgetItem("目录")
            self.recordings_table.setItem(row_idx, 1, type_item)
            
            # 设置信息
            info_item = QTableWidgetItem(dir_item['info'])
            self.recordings_table.setItem(row_idx, 2, info_item)
            
            row_idx += 1
            
            # 检查是否有对应的ZIP文件
            if dir_name in zip_mapping:
                # 添加ZIP文件行，缩进显示
                self.recordings_table.insertRow(row_idx)
                
                # 设置ZIP名称（带缩进）
                zip_name_item = QTableWidgetItem("  └─ " + zip_mapping[dir_name]['name'])
                zip_name_item.setForeground(QColor(0, 120, 215))  # 蓝色
                
                # 在表格项目的userData属性中保存source_dir信息
                zip_name_item.setData(Qt.UserRole, zip_mapping[dir_name]['source_dir'])
                
                self.recordings_table.setItem(row_idx, 0, zip_name_item)
                
                # 设置类型
                zip_type_item = QTableWidgetItem("ZIP文件")
                self.recordings_table.setItem(row_idx, 1, zip_type_item)
                
                # 设置信息
                zip_info_item = QTableWidgetItem(zip_mapping[dir_name]['info'])
                self.recordings_table.setItem(row_idx, 2, zip_info_item)
                
                row_idx += 1
                
                # 从映射中移除已处理的ZIP
                del zip_mapping[dir_name]
        
        # 添加没有对应目录的独立ZIP文件
        for zip_name, zip_info in zip_mapping.items():
            self.recordings_table.insertRow(row_idx)
            
            # 设置名称
            name_item = QTableWidgetItem(zip_info['name'])
            name_item.setForeground(QColor(0, 120, 215))  # 蓝色
            
            # 在表格项目的userData属性中保存source_dir信息
            name_item.setData(Qt.UserRole, zip_info['source_dir'])
            
            self.recordings_table.setItem(row_idx, 0, name_item)
            
            # 设置类型
            type_item = QTableWidgetItem("ZIP文件")
            self.recordings_table.setItem(row_idx, 1, type_item)
            
            # 设置信息
            info_item = QTableWidgetItem(zip_info['info'])
            self.recordings_table.setItem(row_idx, 2, info_item)
            
            row_idx += 1
        
        self.log_message(f"已刷新录制列表，共 {row_idx} 项")
    
    def open_selected_dir(self):
        """打开选中的目录或ZIP文件"""
        selected_rows = self.recordings_table.selectionModel().selectedRows()
        if not selected_rows:
            self.log_message("请先选择一项", error=True)
            return
            
        row = selected_rows[0].row()
        item = self.recordings_table.item(row, 0)
        item_name = item.text().strip()
        item_type = self.recordings_table.item(row, 1).text()
        
        # 获取文件的源目录（如果有）
        source_dir = item.data(Qt.UserRole)
        if source_dir is None:
            source_dir = self.data_storage.base_dir
        
        # 去掉缩进字符如果有的话
        if "└─" in item_name:
            item_name = item_name.split("└─")[1].strip()
        
        # 构建完整路径
        path = os.path.join(source_dir, item_name)
        
        # 检查路径是否存在
        if not os.path.exists(path):
            self.log_message(f"路径不存在: {path}", error=True)
            return
            
        # 使用系统文件管理器打开
        try:
            if sys.platform == 'win32':
                os.startfile(path)
            elif sys.platform == 'darwin':  # macOS
                os.system(f'open "{path}"')
            else:  # Linux
                os.system(f'xdg-open "{path}"')
            
            self.log_message(f"已打开: {path}")
        except Exception as e:
            self.log_message(f"打开路径时出错: {str(e)}", error=True)
    
    def create_zip_for_selected(self):
        """为选中的目录创建ZIP归档"""
        # 检查是否已经在压缩或录制中
        if self.is_compressing:
            self.log_message("另一个ZIP压缩任务正在进行中，请等待完成", error=True)
            return
            
        if self.is_recording:
            self.log_message("正在录制数据，无法同时进行ZIP压缩", error=True)
            return
            
        selected_rows = self.recordings_table.selectionModel().selectedRows()
        if not selected_rows:
            self.log_message("请先选择一项", error=True)
            return
            
        row = selected_rows[0].row()
        item_name = self.recordings_table.item(row, 0).text()
        item_type = self.recordings_table.item(row, 1).text()
        
        # 只能为目录创建ZIP
        if "ZIP" in item_type:
            self.log_message("选择的项目已经是ZIP文件", error=True)
            return
            
        # 构建完整路径
        dir_path = os.path.join(self.data_storage.base_dir, item_name)
        
        # 检查路径是否存在
        if not os.path.exists(dir_path) or not os.path.isdir(dir_path):
            self.log_message(f"目录不存在: {dir_path}", error=True)
            return
            
        # 设置压缩状态
        self.is_compressing = True
        self.update_button_states()
            
        # 显示进度条
        self.zip_progress.setValue(0)
        self.zip_progress.setVisible(True)
        
        # 启动异步压缩任务
        self.zip_worker = ZipWorker(
            self.data_storage,
            dir_path,
            compression_level=self.zip_level_combo.currentIndex(),
            delete_after=self.delete_after_zip_check.isChecked(),
            output_dir=self.args.zip_dir  # 传递ZIP输出目录
        )
        self.zip_worker.progress_signal.connect(self.update_zip_progress)
        self.zip_worker.finished_signal.connect(self.on_zip_finished)
        self.zip_worker.error_signal.connect(self.on_zip_error)
        self.zip_worker.start()
        
        self.log_message(f"正在为 {item_name} 创建ZIP归档...")
    
    def delete_selected(self):
        """删除选中的项目"""
        # 检查是否在压缩中
        if self.is_compressing:
            self.log_message("ZIP压缩进行中，请等待完成后再执行删除操作", error=True)
            return
            
        selected_rows = self.recordings_table.selectionModel().selectedRows()
        if not selected_rows:
            self.log_message("请先选择一项", error=True)
            return
            
        row = selected_rows[0].row()
        item = self.recordings_table.item(row, 0)
        item_name = item.text().strip()
        item_type = self.recordings_table.item(row, 1).text()
        
        # 获取文件的源目录（如果有）
        source_dir = item.data(Qt.UserRole)
        if source_dir is None:
            source_dir = self.data_storage.base_dir
        
        # 去掉缩进字符如果有的话
        if "└─" in item_name:
            item_name = item_name.split("└─")[1].strip()
        
        # 构建完整路径
        path = os.path.join(source_dir, item_name)
        
        # 检查路径是否存在
        if not os.path.exists(path):
            self.log_message(f"路径不存在: {path}", error=True)
            return
            
        # 确认删除
        reply = QMessageBox.question(self, '确认删除', 
            f"确定要删除 {item_name} 吗？此操作不可撤销。", 
            QMessageBox.Yes | QMessageBox.No, QMessageBox.No)
            
        if reply == QMessageBox.No:
            return
            
        # 执行删除
        try:
            if os.path.isdir(path):
                shutil.rmtree(path)
                self.log_message(f"已删除目录: {item_name}")
            else:
                os.remove(path)
                self.log_message(f"已删除文件: {item_name}")
                
            # 刷新列表
            self.refresh_recordings_list()
        except Exception as e:
            self.log_message(f"删除时出错: {str(e)}", error=True)
    
    # ZIP压缩回调
    def update_zip_progress(self, value):
        """更新ZIP压缩进度"""
        self.zip_progress.setValue(value)
    
    def on_zip_finished(self, zip_path):
        """ZIP压缩完成回调"""
        self.zip_progress.setValue(100)
        self.log_message(f"ZIP归档创建成功: {os.path.basename(zip_path)}")
        
        # 重置压缩状态
        self.is_compressing = False
        self.update_button_states()
        
        # 重置数据存储状态
        self.data_storage.reset()
        
        # 3秒后隐藏进度条
        QTimer.singleShot(3000, lambda: self.zip_progress.setVisible(False))
        
        # 刷新列表
        self.refresh_recordings_list()
    
    def on_zip_error(self, error_msg):
        """ZIP压缩错误回调"""
        self.zip_progress.setVisible(False)
        self.log_message(f"ZIP归档创建失败: {error_msg}", error=True)
        
        # 重置压缩状态
        self.is_compressing = False
        self.update_button_states()
        
        # 重置数据存储状态
        self.data_storage.reset()
    
    # 设置功能
    def browse_save_dir(self):
        """浏览并选择保存目录"""
        # 如果在录制或压缩中，不允许更改目录
        if self.is_recording or self.is_compressing:
            self.log_message("正在录制或压缩中，无法更改保存目录", error=True)
            return
            
        dir_path = QFileDialog.getExistingDirectory(
            self, "选择保存目录", self.args.save_dir,
            QFileDialog.ShowDirsOnly
        )
        
        if dir_path:
            self.save_dir_edit.setText(dir_path)
            self.args.save_dir = dir_path
            self.data_storage.base_dir = dir_path
            self.log_message(f"保存目录已更改为: {dir_path}")
            
            # 刷新录制列表
            self.refresh_recordings_list()
    
    def browse_zip_dir(self):
        """浏览并选择ZIP保存目录"""
        # 如果在录制或压缩中，不允许更改目录
        if self.is_recording or self.is_compressing:
            self.log_message("正在录制或压缩中，无法更改ZIP保存目录", error=True)
            return
            
        dir_path = QFileDialog.getExistingDirectory(
            self, "选择ZIP保存目录", self.args.zip_dir,
            QFileDialog.ShowDirsOnly
        )
        
        if dir_path:
            self.zip_dir_edit.setText(dir_path)
            self.args.zip_dir = dir_path
            self.log_message(f"ZIP保存目录已更改为: {dir_path}")
    
    def save_settings(self):
        """保存设置"""
        # 如果在录制或压缩中，不允许保存设置
        if self.is_recording:
            self.log_message("正在录制中，无法保存设置", error=True)
            return
            
        # 更新参数
        self.args.save_dir = self.save_dir_edit.text()
        self.args.zip_dir = self.zip_dir_edit.text()
        self.args.auto_zip = self.auto_zip_check.isChecked()
        self.args.delete_after_zip = self.delete_after_zip_check.isChecked()
        self.args.zip_compression = self.zip_level_combo.currentIndex()
        self.args.compress = self.compress_check.isChecked()
        self.args.ignore_missing_topics = self.ignore_missing_topics_check.isChecked()
        
        # 更新数据存储
        self.data_storage.base_dir = self.args.save_dir
        
        self.log_message("设置已保存")
    
    # 数据处理功能
    def browse_source_dir(self):
        """浏览并选择源数据目录"""
        # 如果在处理中，不允许更改目录
        if self.is_processing:
            self.process_log.append("正在处理数据中，无法更改源目录")
            return
            
        dir_path = QFileDialog.getExistingDirectory(
            self, "选择源数据目录", self.source_dir_edit.text(),
            QFileDialog.ShowDirsOnly
        )
        
        if dir_path:
            self.source_dir_edit.setText(dir_path)
    
    def browse_output_dir(self):
        """浏览并选择输出目录"""
        # 如果在处理中，不允许更改目录
        if self.is_processing:
            self.process_log.append("正在处理数据中，无法更改输出目录")
            return
            
        dir_path = QFileDialog.getExistingDirectory(
            self, "选择输出目录", self.output_dir_edit.text(),
            QFileDialog.ShowDirsOnly
        )
        
        if dir_path:
            self.output_dir_edit.setText(dir_path)
    
    def start_data_processing(self):
        """开始数据处理"""
        # 检查是否在录制或压缩中
        if self.is_recording:
            self.process_log.append("正在录制数据中，无法同时进行数据处理")
            return
            
        if self.is_compressing:
            self.process_log.append("ZIP压缩进行中，无法同时进行数据处理")
            return
            
        if self.is_processing:
            self.process_log.append("已有数据处理任务在进行中")
            return
            
        # 获取源目录和输出目录
        source_dir = self.source_dir_edit.text()
        output_dir = self.output_dir_edit.text()
        
        # 检查目录是否存在
        if not os.path.exists(source_dir):
            self.process_log.append(f"源数据目录不存在: {source_dir}")
            return
            
        # 确保输出目录存在
        os.makedirs(output_dir, exist_ok=True)
        
        # 获取处理线程数
        max_workers = int(self.process_threads_combo.currentText())
        
        # 是否自动删除失败的处理结果
        delete_failed = self.delete_failed_check.isChecked()
        
        # 清空处理日志
        self.process_log.clear()
        self.process_log.append(f"开始处理数据...")
        self.process_log.append(f"源目录: {source_dir}")
        self.process_log.append(f"输出目录: {output_dir}")
        self.process_log.append(f"处理线程数: {max_workers}")
        self.process_log.append(f"自动删除失败结果: {'是' if delete_failed else '否'}")
        
        # 设置处理状态
        self.is_processing = True
        self.update_button_states()
        
        # 启动异步处理任务
        self.process_worker = DataProcessWorker(source_dir, output_dir, max_workers, delete_failed)
        self.process_worker.progress_signal.connect(self.on_process_progress)
        self.process_worker.finished_signal.connect(self.on_process_finished)
        self.process_worker.start()
    
    def on_process_progress(self, message):
        """处理进度更新"""
        self.process_log.append(message)
        # 滚动到底部
        self.process_log.verticalScrollBar().setValue(self.process_log.verticalScrollBar().maximum())
    
    def on_process_finished(self, success, results):
        """处理完成回调"""
        self.is_processing = False
        self.update_button_states()
        
        if success:
            self.process_log.append("\n====== 数据处理完成 ======")
            
            # 展示处理结果对话框
            dialog = ProcessResultDialog(results, self.output_dir_edit.text(), self)
            dialog.exec_()
        else:
            self.process_log.append("\n====== 数据处理失败 ======")
    
    # 回放功能
    def refresh_playback_dirs(self):
        """刷新可用于回放的处理结果目录"""
        self.playback_dir_combo.clear()
        
        # 检查处理输出目录
        output_dir = os.path.join(self.args.save_dir, "processed", "final_output")
        if os.path.exists(output_dir):
            result_dirs = [d for d in os.listdir(output_dir) 
                        if os.path.isdir(os.path.join(output_dir, d))]
            
            for d in sorted(result_dirs, reverse=True):  # 最新的排在前面
                h5_path = os.path.join(output_dir, d, "record", "aligned_joints.h5")
                if os.path.exists(h5_path):
                    self.playback_dir_combo.addItem(d, h5_path)
        
        # 如果有可用目录，自动选择第一个
        if self.playback_dir_combo.count() > 0:
            self.playback_dir_combo.setCurrentIndex(0)
            self.hdf5_path_edit.setText(self.playback_dir_combo.currentData())
    
    def browse_hdf5_file(self):
        """浏览并选择HDF5文件"""
        file_path, _ = QFileDialog.getOpenFileName(
            self, "选择HDF5文件", self.args.save_dir, "HDF5文件 (*.h5 *.hdf5);;所有文件 (*.*)"
        )
        
        if file_path:
            self.hdf5_path_edit.setText(file_path)
            # 当手动选择文件时，取消下拉选择
            self.playback_dir_combo.setCurrentIndex(-1)
    
    def start_trajectory_playback(self):
        """启动轨迹回放对话框"""
        hdf5_path = ""
        
        # 优先使用手动选择的文件
        if self.hdf5_path_edit.text() and os.path.exists(self.hdf5_path_edit.text()):
            hdf5_path = self.hdf5_path_edit.text()
        # 其次使用下拉列表选择的目录
        elif self.playback_dir_combo.currentData():
            hdf5_path = self.playback_dir_combo.currentData()
        
        if not hdf5_path or not os.path.exists(hdf5_path):
            QMessageBox.warning(self, "错误", "请先选择有效的HDF5文件")
            return
        
        # 创建并显示轨迹回放对话框
        player_dialog = TrajectoryPlayerDialog(hdf5_path, self)
        player_dialog.exec_()
    
    def closeEvent(self, event):
        """关闭窗口事件处理"""
        if self.is_recording:
            self.stop_recording()
            
        # 如果在压缩中，询问是否等待
        if self.is_compressing:
            self.log_message("ZIP压缩正在进行中，请等待完成后再退出", error=True)
            event.ignore()
            return
            
        # 如果在处理数据中，询问是否等待
        if self.is_processing:
            reply = QMessageBox.question(self, '确认退出', 
                "数据处理正在进行中，确定要退出吗？", 
                QMessageBox.Yes | QMessageBox.No, QMessageBox.No)
            
            if reply == QMessageBox.No:
                event.ignore()
                return
            
        # 确保所有任务都已完成
        event.accept()

    def browse_ckpt_path(self):
        """浏览模型检查点路径"""
        options = QFileDialog.Options()
        folder_path = QFileDialog.getExistingDirectory(
            self, "选择模型文件夹", "", options=options)
        if folder_path:
            self.ckpt_path_edit.setText(folder_path)
    
    def load_model(self):
        """加载模型"""
        if not self.args.enable_inference or self.inference_engine is None:
            QMessageBox.warning(self, "警告", "推理引擎未启用")
            return
        
        # 获取模型参数
        ckpt_path = self.ckpt_path_edit.text().strip()
        if not ckpt_path:
            QMessageBox.warning(self, "警告", "请输入模型文件夹路径")
            return
        
        # 获取策略类型
        policy_map = {0: 'act', 1: 'diffusion', 2: 'pi0'}
        policy_type = policy_map[self.policy_type_combo.currentIndex()]
        
        # 获取设备类型
        device_map = {0: 'cuda', 1: 'cpu', 2: 'mps'}
        device = device_map[self.device_combo.currentIndex()]
        
        # 显示加载中状态
        self.inference_info_label.setText("推理状态: 正在加载模型...")
        self.statusbar.showMessage("正在加载模型，请稍候...")
        
        # 禁用加载按钮
        self.load_model_button.setEnabled(False)
        
        # 在后台线程中加载模型
        def _load_model_thread():
            # 初始化ROS接口
            if not hasattr(self.inference_engine, 'aloha_cmd_pub') or self.inference_engine.aloha_cmd_pub is None:
                rate = self.inference_rate_spin.value()
                self.inference_engine.init_ros(rate)
            
            # 加载模型
            success = self.inference_engine.load_model(
                policy_type=policy_type,
                ckpt_path=ckpt_path,
                device=device
            )
            
            # 更新界面状态
            if success:
                self.inference_info_label.setText("推理状态: 模型加载成功")
                self.statusbar.showMessage("模型加载成功")
                
                # 启用推理控制按钮
                self.start_inference_button.setEnabled(True)
                self.single_step_button.setEnabled(True)
            else:
                self.inference_info_label.setText("推理状态: 模型加载失败")
                self.statusbar.showMessage("模型加载失败")
            
            # 重新启用加载按钮
            self.load_model_button.setEnabled(True)
        
        # 启动加载线程
        thread = threading.Thread(target=_load_model_thread)
        thread.daemon = True
        thread.start()
    
    def start_inference(self):
        """开始推理"""
        if not self.args.enable_inference or self.inference_engine is None:
            return
        
        if not self.inference_engine.is_initialized:
            QMessageBox.warning(self, "警告", "请先加载模型")
            return
        
        # 设置推理引擎参数
        rate = self.inference_rate_spin.value()
        
        # 是否同时录制
        if self.record_inference_check.isChecked():
            # 启动录制
            self.start_recording()
        
        # 禁用控制按钮
        self.start_inference_button.setEnabled(False)
        self.load_model_button.setEnabled(False)
        self.stop_inference_button.setEnabled(True)
        self.single_step_button.setEnabled(False)
        
        # 更新状态
        self.inference_info_label.setText("推理状态: 正在推理")
        self.statusbar.showMessage("推理已启动")
        
        # 启动推理线程
        def _inference_thread():
            # 启动推理
            self.inference_engine.run_inference(single_step=False)
            
            # 推理完成后更新状态
            self.inference_info_label.setText("推理状态: 推理已停止")
            self.statusbar.showMessage("推理已停止")
            
            # 恢复按钮状态
            self.start_inference_button.setEnabled(True)
            self.load_model_button.setEnabled(True)
            self.stop_inference_button.setEnabled(False)
            self.single_step_button.setEnabled(True)
            
            # 如果在录制，停止录制
            if self.record_inference_check.isChecked() and self.is_recording:
                self.stop_recording()
        
        # 启动线程
        self.inference_thread = threading.Thread(target=_inference_thread)
        self.inference_thread.daemon = True
        self.inference_thread.start()
        
        # 如果设置了超时
        timeout = self.timeout_spin.value()
        if timeout > 0:
            # 创建超时定时器
            def _timeout_handler():
                if self.inference_engine.is_running:
                    self.stop_inference()
                    self.statusbar.showMessage(f"推理已自动停止(达到{timeout}秒超时时间)")
            
            QTimer.singleShot(timeout * 1000, _timeout_handler)
    
    def stop_inference(self):
        """停止推理"""
        if not self.args.enable_inference or self.inference_engine is None:
            return
        
        # 停止推理
        self.inference_engine.stop()
        
        # 恢复按钮状态
        self.start_inference_button.setEnabled(True)
        self.load_model_button.setEnabled(True)
        self.stop_inference_button.setEnabled(False)
        self.single_step_button.setEnabled(True)
        
        # 如果在录制，停止录制
        if self.record_inference_check.isChecked() and self.is_recording:
            self.stop_recording()
    
    def single_step_inference(self):
        """单步推理"""
        if not self.args.enable_inference or self.inference_engine is None:
            return
        
        if not self.inference_engine.is_initialized:
            QMessageBox.warning(self, "警告", "请先加载模型")
            return
        
        # 禁用单步按钮，防止重复点击
        self.single_step_button.setEnabled(False)
        
        # 更新状态
        self.inference_info_label.setText("推理状态: 执行单步推理")
        
        # 执行单步推理
        def _single_step_thread():
            start_time = time.time()
            success = self.inference_engine.run_inference(single_step=True)
            inference_time = (time.time() - start_time) * 1000  # 转换为毫秒
            
            # 更新状态
            if success:
                self.inference_info_label.setText("推理状态: 单步推理成功")
                self.inference_time_label.setText(f"推理时间: {inference_time:.2f} ms")
            else:
                self.inference_info_label.setText("推理状态: 单步推理失败")
            
            # 恢复按钮状态
            self.single_step_button.setEnabled(True)
        
        # 启动线程
        thread = threading.Thread(target=_single_step_thread)
        thread.daemon = True
        thread.start()
    
    def update_display(self):
        """更新显示"""
        # 更新记录状态
        try:
            self.update_status()
            
            # 更新按钮状态
            self.update_button_states()
            
        except Exception as e:
            self.log_message(f"更新显示时出错: {str(e)}", error=True)
            import traceback
            traceback.print_exc()

    def init_workers(self):
        """初始化工作线程"""
        # ... existing code ...
        pass

    # ... existing code ...

    # ... new methods ...

    # ... existing code ... 