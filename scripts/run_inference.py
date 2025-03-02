#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
带推理功能的ROS数据记录系统启动脚本
"""

import os
import sys
import argparse
from ros_data_recorder.main import main as recorder_main

# add
def parse_args():
    """解析命令行参数"""
    parser = argparse.ArgumentParser(description='启动带推理功能的ROS数据记录系统')
    
    # 基础配置
    parser.add_argument('--save_dir', type=str, default="./recorded_data",
                      help='数据保存目录')
    
    # 推理配置
    parser.add_argument('--policy_type', type=str, default='act',
                      choices=['act', 'diffusion', 'pi0'],
                      help='推理策略类型, 可选值: act, diffusion, pi0')
    parser.add_argument('--inference_rate', type=float, default=50.0,
                      help='推理频率(Hz)')
    parser.add_argument('--device', type=str, default='cuda',
                      choices=['cuda', 'cpu', 'mps'],
                      help='推理设备类型, 可选值: cuda, cpu, mps')
    parser.add_argument('--ckpt_path', type=str, 
                      default='',
                      help='模型检查点路径')
    parser.add_argument('--record_inference', action='store_true',
                      help='是否同时录制推理过程中的数据')
    
    args = parser.parse_args()
    return args

def main():
    """主函数"""
    args = parse_args()
    
    # 准备参数列表
    sys_args = []
    sys_args.append('ros_data_recorder_gui')
    
    # 添加启用推理的参数
    sys_args.append('--enable_inference')
    
    # 添加其他参数
    if args.save_dir:
        sys_args.extend(['--save_dir', args.save_dir])
    
    if args.policy_type:
        sys_args.extend(['--policy_type', args.policy_type])
        
    if args.inference_rate:
        sys_args.extend(['--inference_rate', str(args.inference_rate)])
        
    if args.device:
        sys_args.extend(['--device', args.device])
        
    if args.ckpt_path:
        sys_args.extend(['--ckpt_path', args.ckpt_path])
        
    if args.record_inference:
        sys_args.append('--record_inference')
    
    # 打印参数
    print("启动参数:")
    for arg in sys_args:
        print(f"  {arg}")
        
    # 启动记录系统
    sys.argv = sys_args
    recorder_main()

if __name__ == '__main__':
    main() 