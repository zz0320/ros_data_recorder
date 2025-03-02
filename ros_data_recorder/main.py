#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
主入口模块，包含程序的运行入口
"""

import sys
from PyQt5.QtWidgets import QApplication
from .config import get_arguments
from .ui.main_window import ROSRecorderGUI

def main():
    """主函数"""
    # 解析参数
    args = get_arguments()
    
    # 确保所需的库已导入
    try:
        import h5py
        from PyQt5.QtCore import QMetaObject, Q_ARG
        import cv2
    except ImportError as e:
        print(f"错误: 缺少必要的依赖库: {e}")
        print("请安装所需依赖: pip install h5py PyQt5 opencv-python")
        sys.exit(1)
    
    # 创建QT应用
    app = QApplication(sys.argv)
    app.setStyle('Fusion')  # 使用Fusion风格，跨平台一致
    
    # 创建主窗口
    window = ROSRecorderGUI(args)
    window.show()
    
    # 运行应用
    sys.exit(app.exec_())

if __name__ == '__main__':
    main() 