#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
配置处理模块，负责解析命令行参数和管理配置
"""

import argparse

def get_arguments():
    """解析命令行参数"""
    parser = argparse.ArgumentParser(description='ROS数据记录与处理系统参数配置')
    
    # 基础配置
    parser.add_argument('--save_dir', type=str, default="./recorded_data",
                      help='数据保存目录')
    parser.add_argument('--zip_dir', type=str, default="./recorded_data_zip",
                      help='ZIP文件保存目录，默认与数据保存目录相同')
    parser.add_argument('--frame_rate', type=int, default=30,
                      help='状态刷新帧率')
    parser.add_argument('--compress', action='store_true', default=True,
                      help='是否使用压缩图像话题')
    
    # 录制配置
    parser.add_argument('--auto_zip', action='store_true', default=True,
                      help='是否自动将录制数据打包为ZIP')
    parser.add_argument('--delete_after_zip', action='store_true', default=False,
                      help='创建ZIP后是否删除原始数据目录')
    parser.add_argument('--zip_compression', type=int, default=6,
                      help='ZIP压缩级别(0-9)，0表示不压缩，9表示最大压缩')
    parser.add_argument('--ignore_missing_topics', action='store_false', default=False,
                      help='忽略缺失的夹爪话题，允许在部分话题缺失时进行录制')
    
    # 话题配置
    parser.add_argument('--img_front_topic', type=str,
                      default='/camera_f/color/image_raw/compressed',
                      help='前方相机话题')
    parser.add_argument('--img_left_topic', type=str,
                      default='/camera_l/color/image_raw/compressed',
                      help='左侧相机话题')
    parser.add_argument('--img_right_topic', type=str,
                      default='/camera_r/color/image_raw/compressed',
                      help='右侧相机话题')
    parser.add_argument('--puppet_state_topic', type=str,
                      default='/puppet',
                      help='机器人状态话题')
    parser.add_argument('--gripper_left_topic', type=str,
                      default='/gripper1_position_mm_upsample',
                      help='左夹爪话题')
    parser.add_argument('--gripper_right_topic', type=str,
                      default='/gripper2_position_mm_upsample',
                      help='右夹爪话题')
    parser.add_argument('--gripper_action_topic', type=str,
                      default='/gripper_action',
                      help='夹爪控制指令话题')
    
    # 推理配置
    parser.add_argument('--enable_inference', action='store_true', default=False,
                      help='是否启用模型推理功能')
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
    parser.add_argument('--record_inference', action='store_true', default=False,
                      help='是否同时录制推理过程中的数据')
    parser.add_argument('--inference_timeout', type=int, default=0,
                      help='推理超时时间(秒)，0表示不限制')
    
    return parser.parse_args() 