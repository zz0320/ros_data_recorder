#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
对话框模块，包含各种对话框类
"""

import os
import sys
import h5py
import time
from datetime import datetime
from PyQt5.QtWidgets import (QDialog, QVBoxLayout, QHBoxLayout, QGroupBox, QLabel, 
                          QPushButton, QProgressBar, QTextEdit, QTableWidget, 
                          QTableWidgetItem, QHeaderView, QSizePolicy, QComboBox, 
                          QCheckBox, QMessageBox, QLineEdit, QFrame)
from PyQt5.QtCore import Qt, QTimer, pyqtSignal, pyqtSlot
from PyQt5.QtGui import QColor, QFont, QPixmap, QIcon, QFontDatabase, QTextCursor, QFont

from PyQt5.QtWidgets import (QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, 
                          QPushButton, QLabel, QTableWidget, QTableWidgetItem, QGroupBox, 
                          QProgressBar, QTextEdit, QTabWidget, QFileDialog, QGridLayout,
                          QSpacerItem, QSizePolicy, QCheckBox, QComboBox, QScrollArea,
                          QFrame, QHeaderView, QSplitter, QMessageBox, QListWidget,
                          QListWidgetItem, QDialog, QRadioButton, QButtonGroup, QLineEdit, QApplication, QGridLayout)
from PyQt5.QtCore import QTimer, Qt, pyqtSignal, pyqtSlot, QThread
from PyQt5.QtGui import QColor, QFont, QPixmap, QIcon, QFontDatabase, QTextCursor, QFont

from ..workers.playback_thread import PlaybackThread

class TrajectoryPlayerDialog(QDialog):
    """轨迹回放对话框"""
    def __init__(self, hdf5_path=None, parent=None):
        super().__init__(parent)
        self.hdf5_path = hdf5_path
        self.is_playing = False
        self.current_frame = 0
        self.total_frames = 0
        self.data = None
        self.playback_thread = None
        self.camera_base = None
        self.init_ui()
        
    def init_ui(self):
        """初始化用户界面"""
        self.setWindowTitle("轨迹回放控制器")
        self.setMinimumSize(1000, 700)  # 增大窗口以容纳图像显示
        
        layout = QVBoxLayout(self)
        
        # 文件信息区域
        file_group = QGroupBox("文件信息")
        file_layout = QGridLayout(file_group)
        
        file_layout.addWidget(QLabel("HDF5文件:"), 0, 0)
        self.file_path_label = QLabel(self.hdf5_path or "未选择文件")
        self.file_path_label.setWordWrap(True)
        file_layout.addWidget(self.file_path_label, 0, 1, 1, 2)
        
        browse_button = QPushButton("浏览...")
        browse_button.clicked.connect(self.browse_file)
        file_layout.addWidget(browse_button, 0, 3)
        
        file_layout.addWidget(QLabel("数据信息:"), 1, 0)
        self.data_info_label = QLabel("未加载")
        file_layout.addWidget(self.data_info_label, 1, 1, 1, 3)
        
        layout.addWidget(file_group)
        
        # 图像显示区域 (新增)
        image_group = QGroupBox("相机图像")
        image_layout = QHBoxLayout(image_group)
        
        # 三个图像标签，用于显示相机图像
        self.image_labels = {}
        for cam_name in ['前方相机', '左侧相机', '右侧相机']:
            cam_frame = QFrame()
            cam_frame.setFrameShape(QFrame.Box)
            cam_frame.setLineWidth(1)
            cam_layout = QVBoxLayout(cam_frame)
            
            # 标题
            title_label = QLabel(cam_name)
            title_label.setAlignment(Qt.AlignCenter)
            cam_layout.addWidget(title_label)
            
            # 图像标签
            img_label = QLabel()
            img_label.setAlignment(Qt.AlignCenter)
            img_label.setMinimumSize(320, 240)
            img_label.setText("无图像")
            cam_layout.addWidget(img_label)
            
            image_layout.addWidget(cam_frame)
            self.image_labels[cam_name] = img_label
        
        layout.addWidget(image_group)
        
        # 回放控制区域
        control_group = QGroupBox("回放控制")
        control_layout = QVBoxLayout(control_group)
        
        # 进度条
        progress_layout = QHBoxLayout()
        progress_layout.addWidget(QLabel("进度:"))
        self.progress_bar = QProgressBar()
        self.progress_bar.setRange(0, 100)
        self.progress_bar.setValue(0)
        progress_layout.addWidget(self.progress_bar)
        self.frame_label = QLabel("0/0")
        progress_layout.addWidget(self.frame_label)
        control_layout.addLayout(progress_layout)
        
        # 按钮区域
        buttons_layout = QHBoxLayout()
        
        self.play_button = QPushButton("播放")
        self.play_button.setIcon(QIcon.fromTheme("media-playback-start"))
        self.play_button.clicked.connect(self.toggle_play)
        self.play_button.setEnabled(False)
        buttons_layout.addWidget(self.play_button)
        
        self.stop_button = QPushButton("停止")
        self.stop_button.setIcon(QIcon.fromTheme("media-playback-stop"))
        self.stop_button.clicked.connect(self.stop_playback)
        self.stop_button.setEnabled(False)
        buttons_layout.addWidget(self.stop_button)
        
        self.load_button = QPushButton("加载数据")
        self.load_button.clicked.connect(self.load_data)
        self.load_button.setEnabled(bool(self.hdf5_path))
        buttons_layout.addWidget(self.load_button)
        
        control_layout.addLayout(buttons_layout)
        
        # 回放选项
        options_layout = QHBoxLayout()
        options_layout.addWidget(QLabel("回放速率:"))
        
        self.rate_combo = QComboBox()
        rates = ["25Hz", "30Hz", "50Hz", "60Hz", "100Hz"]
        self.rate_combo.addItems(rates)
        self.rate_combo.setCurrentText("60Hz")
        options_layout.addWidget(self.rate_combo)
        
        options_layout.addWidget(QLabel("显示图像:"))
        self.show_images_check = QCheckBox()
        self.show_images_check.setChecked(True)
        options_layout.addWidget(self.show_images_check)
        
        control_layout.addLayout(options_layout)
        
        layout.addWidget(control_group)
        
        # 日志区域
        log_group = QGroupBox("回放日志")
        log_layout = QVBoxLayout(log_group)
        
        self.log_text = QTextEdit()
        self.log_text.setReadOnly(True)
        log_layout.addWidget(self.log_text)
        
        layout.addWidget(log_group)
        
        # 底部按钮
        buttons_layout = QHBoxLayout()
        close_button = QPushButton("关闭")
        close_button.clicked.connect(self.close)
        buttons_layout.addWidget(close_button)
        
        layout.addLayout(buttons_layout)
    
    def browse_file(self):
        """浏览并选择HDF5文件"""
        if self.is_playing:
            self.log_message("请先停止回放再更换文件", error=True)
            return
            
        file_path, _ = QFileDialog.getOpenFileName(
            self, "选择HDF5文件", "", "HDF5文件 (*.h5 *.hdf5);;所有文件 (*.*)"
        )
        
        if file_path:
            self.hdf5_path = file_path
            self.file_path_label.setText(file_path)
            self.load_button.setEnabled(True)
            self.log_message(f"已选择文件: {file_path}")
    
    def load_data(self):
        """加载HDF5数据"""
        if not self.hdf5_path or not os.path.exists(self.hdf5_path):
            self.log_message("请先选择有效的HDF5文件", error=True)
            return
            
        try:
            self.log_message("正在加载数据...")
            
            # 加载HDF5文件
            with h5py.File(self.hdf5_path, 'r') as f:
                # 获取动作数据
                actions_joint = f['action/joint/position'][:]
                actions_gripper = f['action/effector/position'][:]
                
                # 获取状态数据
                states_joint = f['state/joint/position'][:]
                states_gripper = f['state/effector/position'][:]
                
                # 获取时间戳
                timestamps = f['timestamp'][:]
                
                # 保存数据
                self.data = {
                    'actions_joint': actions_joint,
                    'actions_gripper': actions_gripper,
                    'states_joint': states_joint,
                    'states_gripper': states_gripper,
                    'timestamps': timestamps
                }
                # 添加数据验证日志
                self.log_message(f"读取动作数据: 形状={actions_joint.shape}, 范围=[{actions_joint.min()}, {actions_joint.max()}]")
                self.log_message(f"读取夹爪数据: 形状={actions_gripper.shape}, 范围=[{actions_gripper.min()}, {actions_gripper.max()}]")
                            
            # 计算总帧数
            self.total_frames = len(self.data['timestamps'])
            
            # 检查图像帧数
            self.data_dir = os.path.dirname(os.path.dirname(self.hdf5_path))
            self.camera_base = os.path.join(self.data_dir, 'camera')
            
            if os.path.exists(self.camera_base):
                available_frames = len([d for d in os.listdir(self.camera_base) 
                                     if os.path.isdir(os.path.join(self.camera_base, d))])
                if available_frames != self.total_frames:
                    self.log_message(f"警告: 图像帧数 ({available_frames}) "
                                   f"与轨迹帧数 ({self.total_frames}) 不匹配", error=True)
            else:
                self.log_message("警告: 未找到对应的相机图像目录", error=True)
            
            # 更新UI
            self.data_info_label.setText(f"总帧数: {self.total_frames}, "
                                        f"动作维度: {actions_joint.shape}")
            self.progress_bar.setRange(0, self.total_frames)
            self.progress_bar.setValue(0)
            self.frame_label.setText(f"0/{self.total_frames}")
            
            # 启用控制按钮
            self.play_button.setEnabled(True)
            
            self.log_message(f"数据加载成功 - 总帧数: {self.total_frames}")
            
        except Exception as e:
            self.log_message(f"加载数据失败: {str(e)}", error=True)
    
    def toggle_play(self):
        """开始或暂停回放"""
        if not self.data:
            self.log_message("请先加载数据", error=True)
            return
            
        if self.is_playing:
            # 暂停回放
            if self.playback_thread:
                self.playback_thread.is_playing = False
            self.is_playing = False
            self.play_button.setText("继续")
            self.log_message("回放已暂停")
        else:
            # 开始/继续回放
            self.is_playing = True
            self.play_button.setText("暂停")
            self.stop_button.setEnabled(True)
            
            # 如果已经到达末尾，重新开始
            if self.current_frame >= self.total_frames:
                self.current_frame = 0
                self.progress_bar.setValue(0)
                self.frame_label.setText(f"0/{self.total_frames}")
            
            # 使用QThread替代Python线程
            if not self.playback_thread or not self.playback_thread.isRunning():
                # 获取回放速率
                rate_text = self.rate_combo.currentText()
                hz = int(rate_text.replace("Hz", ""))
                
                # 创建并启动线程
                self.playback_thread = PlaybackThread(
                    self.data, 
                    hz, 
                    self.show_images_check.isChecked(),
                    self.camera_base
                )
                
                # 连接信号
                self.playback_thread.progress_signal.connect(self.progress_bar.setValue)
                self.playback_thread.text_signal.connect(self.frame_label.setText)
                self.playback_thread.log_signal.connect(self.log_message)
                self.playback_thread.completed_signal.connect(self.playback_completed)
                # 连接图像信号
                self.playback_thread.image_signal.connect(self.update_image)
                
                # 设置初始帧
                self.playback_thread.current_frame = self.current_frame
                
                # 启动线程
                self.playback_thread.start()
                
            self.log_message("开始回放")
    
    def stop_playback(self):
        """停止回放"""
        if self.playback_thread and self.playback_thread.isRunning():
            self.playback_thread.is_playing = False
            self.playback_thread.requestInterruption()
            self.playback_thread.wait(1000)  # 等待线程结束，最多1秒
            
        self.is_playing = False
        self.current_frame = 0
        self.progress_bar.setValue(0)
        self.frame_label.setText(f"0/{self.total_frames}")
        self.play_button.setText("播放")
        self.stop_button.setEnabled(False)
        
        # 清空图像显示
        for img_label in self.image_labels.values():
            img_label.setText("无图像")
            img_label.setPixmap(QPixmap())
        
        self.log_message("回放已停止")
    
    @pyqtSlot()
    def playback_completed(self):
        """回放完成处理"""
        self.is_playing = False
        self.play_button.setText("播放")
        self.stop_button.setEnabled(False)
        self.current_frame = self.total_frames
    
    @pyqtSlot(str, QPixmap)
    def update_image(self, camera_name, pixmap):
        """更新相机图像"""
        if camera_name in self.image_labels:
            self.image_labels[camera_name].setPixmap(pixmap)
    
    def log_message(self, message, error=False, update=False):
        """添加日志消息"""
        timestamp = datetime.now().strftime('%H:%M:%S')
        
        if error:
            log_entry = f"<span style='color:red'>[{timestamp}] ERROR: {message}</span>"
        else:
            log_entry = f"[{timestamp}] {message}"
        
        # 对于更新类型的消息，替换最后一行
        if update and self.log_text.document().blockCount() > 0:
            cursor = self.log_text.textCursor()
            cursor.movePosition(cursor.End)
            cursor.movePosition(cursor.StartOfBlock, cursor.KeepAnchor)
            cursor.removeSelectedText()
            cursor.deletePreviousChar()  # 删除换行符
            self.log_text.setTextCursor(cursor)
            self.log_text.append(log_entry)
        else:
            self.log_text.append(log_entry)
        
        # 滚动到底部
        self.log_text.verticalScrollBar().setValue(self.log_text.verticalScrollBar().maximum())
        
        # 同时在控制台输出
        print(f"[{timestamp}] {'ERROR: ' if error else ''}{message}")
    
    def closeEvent(self, event):
        """处理窗口关闭事件"""
        # 停止回放线程
        if self.playback_thread and self.playback_thread.isRunning():
            self.playback_thread.is_playing = False
            self.playback_thread.requestInterruption()
            if not self.playback_thread.wait(2000):  # 等待2秒
                print("警告: 无法正常终止回放线程")
        
        # 接受关闭事件
        event.accept()


class ProcessResultDialog(QDialog):
    """处理结果展示对话框"""
    def __init__(self, results, output_dir, parent=None):
        super().__init__(parent)
        self.results = results
        self.output_dir = output_dir
        self.init_ui()
        
    def init_ui(self):
        self.setWindowTitle("数据处理结果")
        self.setMinimumSize(600, 400)
        
        layout = QVBoxLayout(self)
        
        # 标题
        title_label = QLabel("组帧同步处理结果")
        title_label.setFont(QFont('Arial', 14, QFont.Bold))
        title_label.setAlignment(Qt.AlignCenter)
        layout.addWidget(title_label)
        
        # 结果表格
        self.result_table = QTableWidget(len(self.results), 4)
        self.result_table.setHorizontalHeaderLabels(["目录名", "状态", "帧数", "操作"])
        self.result_table.setSelectionBehavior(QTableWidget.SelectRows)
        self.result_table.setSelectionMode(QTableWidget.SingleSelection)
        self.result_table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        
        for i, result in enumerate(self.results):
            dir_name = result.get('dir_name', '')
            success = result.get('success', False)
            frame_count = result.get('frame_count', 0)
            
            # 目录名
            self.result_table.setItem(i, 0, QTableWidgetItem(dir_name))
            
            # 状态
            status_item = QTableWidgetItem("成功" if success else "失败")
            status_item.setForeground(QColor("green") if success else QColor("red"))
            self.result_table.setItem(i, 1, status_item)
            
            # 帧数
            self.result_table.setItem(i, 2, QTableWidgetItem(str(frame_count)))
            
            # 操作按钮单元格
            cell_widget = QWidget()
            cell_layout = QHBoxLayout(cell_widget)
            cell_layout.setContentsMargins(2, 2, 2, 2)
            
            # 查看按钮
            view_button = QPushButton("查看")
            view_button.setEnabled(success)
            view_button.clicked.connect(lambda checked, row=i: self.view_result(row))
            cell_layout.addWidget(view_button)
            
            # 删除按钮
            delete_button = QPushButton("删除")
            delete_button.setEnabled(success)
            delete_button.clicked.connect(lambda checked, row=i: self.delete_result(row))
            cell_layout.addWidget(delete_button)
            
            self.result_table.setCellWidget(i, 3, cell_widget)
        
        layout.addWidget(self.result_table)
        
        # 摘要信息
        summary_frame = QFrame()
        summary_frame.setFrameShape(QFrame.StyledPanel)
        summary_layout = QHBoxLayout(summary_frame)
        
        successful = sum(1 for r in self.results if r.get('success', False))
        total_frames = sum(r.get('frame_count', 0) for r in self.results)
        
        summary_layout.addWidget(QLabel(f"成功处理: {successful}/{len(self.results)}"))
        summary_layout.addWidget(QLabel(f"总帧数: {total_frames}"))
        summary_layout.addWidget(QLabel(f"保存目录: {self.output_dir}"))
        
        layout.addWidget(summary_frame)
        
        # 按钮区域
        button_layout = QHBoxLayout()
        
        open_dir_button = QPushButton("打开输出目录")
        open_dir_button.clicked.connect(self.open_output_dir)
        button_layout.addWidget(open_dir_button)
        
        close_button = QPushButton("关闭")
        close_button.clicked.connect(self.accept)
        button_layout.addWidget(close_button)
        
        layout.addLayout(button_layout)
    
    def view_result(self, row):
        """查看处理结果"""
        if row < 0 or row >= len(self.results):
            return
            
        result = self.results[row]
        if not result.get('success', False):
            return
            
        dir_name = result.get('dir_name', '')
        final_output_dir = os.path.join(self.output_dir, "final_output", dir_name)
        
        if not os.path.exists(final_output_dir):
            QMessageBox.warning(self, "错误", f"找不到输出目录: {final_output_dir}")
            return
            
        # 使用系统文件管理器打开
        try:
            if sys.platform == 'win32':
                os.startfile(final_output_dir)
            elif sys.platform == 'darwin':  # macOS
                os.system(f'open "{final_output_dir}"')
            else:  # Linux
                os.system(f'xdg-open "{final_output_dir}"')
        except Exception as e:
            QMessageBox.warning(self, "错误", f"打开目录失败: {str(e)}")
    
    def delete_result(self, row):
        """删除处理结果"""
        if row < 0 or row >= len(self.results):
            return
            
        result = self.results[row]
        if not result.get('success', False):
            return
            
        dir_name = result.get('dir_name', '')
        final_output_dir = os.path.join(self.output_dir, "final_output", dir_name)
        
        if not os.path.exists(final_output_dir):
            QMessageBox.warning(self, "错误", f"找不到输出目录: {final_output_dir}")
            return
            
        # 确认删除
        reply = QMessageBox.question(self, "确认删除", 
            f"确定要删除 {dir_name} 的处理结果吗？此操作不可撤销。",
            QMessageBox.Yes | QMessageBox.No, QMessageBox.No)
            
        if reply != QMessageBox.Yes:
            return
            
        # 执行删除
        try:
            shutil.rmtree(final_output_dir)
            QMessageBox.information(self, "成功", f"已删除 {dir_name} 的处理结果")
            
            # 更新表格
            status_item = self.result_table.item(row, 1)
            status_item.setText("已删除")
            status_item.setForeground(QColor("gray"))
            
            # 禁用操作按钮
            cell_widget = self.result_table.cellWidget(row, 3)
            for i in range(cell_widget.layout().count()):
                widget = cell_widget.layout().itemAt(i).widget()
                if isinstance(widget, QPushButton):
                    widget.setEnabled(False)
            
            # 更新结果数据
            self.results[row]['success'] = False
            self.results[row]['frame_count'] = 0
        except Exception as e:
            QMessageBox.warning(self, "错误", f"删除失败: {str(e)}")
    
    def open_output_dir(self):
        """打开输出目录"""
        output_dir = os.path.join(self.output_dir, "final_output")
        if not os.path.exists(output_dir):
            QMessageBox.warning(self, "错误", f"找不到输出目录: {output_dir}")
            return
            
        try:
            if sys.platform == 'win32':
                os.startfile(output_dir)
            elif sys.platform == 'darwin':  # macOS
                os.system(f'open "{output_dir}"')
            else:  # Linux
                os.system(f'xdg-open "{output_dir}"')
        except Exception as e:
            QMessageBox.warning(self, "错误", f"打开目录失败: {str(e)}") 