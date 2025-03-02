#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
推理引擎模块，负责加载模型和执行推理
"""

import os
import time
import numpy as np
import torch
import cv2
import rospy
from collections import deque
from std_msgs.msg import Header
from sensor_msgs.msg import JointState, CompressedImage
from cv_bridge import CvBridge

class PolicyFactory:
    """策略工厂类，用于创建不同类型的策略模型"""
    
    @staticmethod
    def create_policy(policy_type, ckpt_path, device):
        """创建策略模型
        
        Args:
            policy_type (str): 策略类型，可选值为 'act', 'diffusion', 'pi0'
            ckpt_path (str): 模型文件夹路径
            device (str): 设备类型，例如 'cuda', 'cpu', 'mps'
            
        Returns:
            Policy: 策略模型实例或None
        """
        try:
            # 确保路径存在
            ckpt_path = os.path.abspath(ckpt_path)
            if not os.path.exists(ckpt_path):
                raise FileNotFoundError(f"检查点路径不存在: {ckpt_path}")
            
            # 确保输入是文件夹
            if not os.path.isdir(ckpt_path):
                raise ValueError(f"检查点路径必须是文件夹: {ckpt_path}")
            
            # 根据策略类型导入相应的模型
            if policy_type == 'act':
                from lerobot.common.policies.act.modeling_act import ACTPolicy
                policy_class = ACTPolicy
            elif policy_type == 'diffusion':
                from lerobot.common.policies.diffusion.modeling_diffusion import DiffusionPolicy
                policy_class = DiffusionPolicy
            elif policy_type == 'pi0':
                from lerobot.common.policies.pi0.modeling_pi0 import PI0Policy
                policy_class = PI0Policy
            else:
                raise ValueError(f"不支持的策略类型: {policy_type}")
            
            # 加载预训练模型
            policy = policy_class.from_pretrained(
                pretrained_name_or_path=ckpt_path,
                local_files_only=True,
                map_location=device
            )
            
            # 将模型迁移到指定设备
            policy.to(device)
            return policy
            
        except ImportError as e:
            rospy.logerr(f"导入模型库时出错: {str(e)}")
            rospy.logerr("请确保已安装 lerobot 库: pip install lerobot")
            return None
        except Exception as e:
            rospy.logerr(f"创建策略模型时出错: {str(e)}")
            return None


class InferenceEngine:
    """推理引擎类，负责加载模型和执行推理"""
    
    def __init__(self, args=None):
        """初始化推理引擎
        
        Args:
            args: 参数对象，包含模型和设备配置
        """
        self.args = args
        self.policy = None
        self.device = 'cuda' if torch.cuda.is_available() else 'cpu'
        self.bridge = CvBridge()
        
        # 初始化数据队列
        self._init_queues()
        
        # 初始化模型参数
        self.policy_type = 'act'  # 默认使用ACT策略
        self.ckpt_path = None
        self.is_initialized = False
        self.is_running = False
        
        # 发布器
        self.aloha_cmd_pub = None
        self.gripper_action_pub = None
        self.rate = None
    
    def _init_queues(self):
        """初始化数据队列"""
        max_size = 10
        self.img_deques = {
            'left': deque(maxlen=max_size),
            'right': deque(maxlen=max_size),
            'front': deque(maxlen=max_size)
        }
        self.puppet_state_deque = deque(maxlen=max_size)
        self.gripper_left_deque = deque(maxlen=max_size)
        self.gripper_right_deque = deque(maxlen=max_size)
    
    def init_ros(self, rate=50.0):
        """初始化ROS发布器和订阅器
        
        Args:
            rate: 发布频率，单位Hz
        """
        # 初始化发布率
        try:
            self.rate = rospy.Rate(rate)
        except:
            rospy.logwarn(f"无法创建指定频率({rate}Hz)的Rate对象，将使用默认值(50Hz)")
            self.rate = rospy.Rate(50)
        
        # 初始化发布器
        try:
            from sdk.msg import RosControl, AlohaCmd, PuppetState
            self.aloha_cmd_pub = rospy.Publisher('/aloha_cmd', AlohaCmd, queue_size=1)
            self.gripper_action_pub = rospy.Publisher('/gripper_action_pub', JointState, queue_size=1)
            
            # 创建回调函数
            def image_callback(msg, name):
                try:
                    rospy.logdebug(f"接收到{name}相机图像，大小: {len(msg.data)}字节")
                    self.img_deques[name].append(msg)
                except Exception as e:
                    rospy.logerr(f"处理{name}相机图像时出错: {str(e)}")
            
            def puppet_state_callback(msg):
                try:
                    rospy.logdebug(f"接收到机器人状态，左臂: {len(msg.arm_left.position)}个关节, 右臂: {len(msg.arm_right.position)}个关节")
                    self.puppet_state_deque.append(msg)
                except Exception as e:
                    rospy.logerr(f"处理机器人状态时出错: {str(e)}")
            
            def gripper_callback(msg, side):
                try:
                    rospy.logdebug(f"接收到{side}夹爪位置: {msg.position}")
                    if side == "left":
                        self.gripper_left_deque.append(msg)
                    else:
                        self.gripper_right_deque.append(msg)
                except Exception as e:
                    rospy.logerr(f"处理{side}夹爪位置时出错: {str(e)}")
            
            # 订阅话题
            rospy.loginfo("开始订阅相机话题...")
            self.image_subscribers = []
            for name, topic in [
                ('left', '/camera_l/color/image_raw/compressed'),
                ('right', '/camera_r/color/image_raw/compressed'),
                ('front', '/camera_f/color/image_raw/compressed')
            ]:
                sub = rospy.Subscriber(
                    topic, CompressedImage,
                    lambda msg, name=name: image_callback(msg, name),
                    queue_size=1, tcp_nodelay=True
                )
                self.image_subscribers.append(sub)
                rospy.loginfo(f"已订阅{name}相机话题: {topic}")
            
            # 订阅状态话题
            rospy.loginfo("开始订阅状态话题...")
            self.puppet_state_sub = rospy.Subscriber(
                '/puppet', PuppetState,
                puppet_state_callback,
                queue_size=1
            )
            
            self.gripper_left_sub = rospy.Subscriber(
                '/gripper1_position_mm_upsample', JointState,
                lambda msg: gripper_callback(msg, "left"),
                queue_size=1
            )
            
            self.gripper_right_sub = rospy.Subscriber(
                '/gripper2_position_mm_upsample', JointState,
                lambda msg: gripper_callback(msg, "right"),
                queue_size=1
            )
            
            rospy.loginfo("所有ROS话题订阅成功")
            return True
        except ImportError as e:
            rospy.logerr(f"导入ROS消息类型时出错: {str(e)}")
            return False
        except Exception as e:
            rospy.logerr(f"初始化ROS接口时出错: {str(e)}")
            return False
    
    def load_model(self, policy_type='act', ckpt_path=None, device=None):
        """加载模型
        
        Args:
            policy_type: 策略类型，可选值为 'act', 'diffusion', 'pi0'
            ckpt_path: 检查点路径，应该是文件夹路径
            device: 设备类型，如 'cuda', 'cpu' 等
            
        Returns:
            是否成功加载模型
        """
        if device is None:
            device = self.device
        
        if ckpt_path is None:
            rospy.logerr("未指定检查点路径")
            return False
        
        # 处理路径
        ckpt_path = os.path.abspath(ckpt_path)
        if not os.path.exists(ckpt_path):
            rospy.logerr(f"检查点路径不存在: {ckpt_path}")
            return False
        
        # 确保输入是文件夹路径
        if not os.path.isdir(ckpt_path):
            rospy.logerr(f"检查点路径应该是文件夹: {ckpt_path}")
            return False
            
        # 检查是否是以pretrained_model结尾
        if not ckpt_path.endswith('pretrained_model'):
            pretrained_path = os.path.join(ckpt_path, 'pretrained_model')
            if os.path.exists(pretrained_path):
                ckpt_path = pretrained_path
                rospy.loginfo(f"使用子目录 pretrained_model: {ckpt_path}")
        
        self.policy_type = policy_type
        self.ckpt_path = ckpt_path
        
        try:
            # 创建策略模型
            self.policy = PolicyFactory.create_policy(policy_type, ckpt_path, device)
            if self.policy is None:
                return False
            
            self.is_initialized = True
            return True
        except Exception as e:
            rospy.logerr(f"加载模型时出错: {str(e)}")
            self.is_initialized = False
            return False
    
    def get_observation(self):
        """获取当前观测数据
        
        Returns:
            观测数据字典，如果数据不完整则返回None
        """
        # 检查数据是否完整
        if not all(len(deque) > 0 for deque in [
            self.puppet_state_deque,
            self.gripper_left_deque,
            self.gripper_right_deque,
            *self.img_deques.values()
        ]):
            return None
        
        try:
            obs_dict = {}
            
            # 处理状态数据
            puppet_state = self.puppet_state_deque[-1]
            gripper_left = self.gripper_left_deque[-1]
            gripper_right = self.gripper_right_deque[-1]
            
            # 构建状态向量
            state = []
            
            # 机器人臂数据
            left_arm_data = np.array(puppet_state.arm_left.position)
            right_arm_data = np.array(puppet_state.arm_right.position)
            
            # 夹爪数据
            left_gripper_data = np.array(gripper_left.position)[:1]
            right_gripper_data = np.array(gripper_right.position)[:1]
            
            # 合并状态数据
            state.append(torch.from_numpy(left_arm_data))
            state.append(torch.from_numpy(left_gripper_data))
            state.append(torch.from_numpy(right_arm_data))
            state.append(torch.from_numpy(right_gripper_data))
            
            # 组合所有状态数据
            state = torch.cat(state).float()
            obs_dict["observation.state"] = state
            
            # 处理图像数据
            for name, deque in self.img_deques.items():
                msg = deque[-1]
                # 解码压缩图像
                np_arr = np.frombuffer(msg.data, np.uint8)
                img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
                # 转换为torch张量
                img_tensor = torch.from_numpy(img).float()
                obs_dict[f"observation.images.{name}"] = img_tensor
            
            return obs_dict
        except Exception as e:
            rospy.logerr(f"获取观测数据时出错: {str(e)}")
            return None
    
    def publish_actions(self, actions):
        """发布动作指令
        
        Args:
            actions: 动作数组，包含左右臂和夹爪的控制指令
            
        Returns:
            是否成功发布
        """
        if not self.is_initialized:
            rospy.logerr("推理引擎未初始化")
            return False
        
        if self.aloha_cmd_pub is None or self.gripper_action_pub is None:
            rospy.logerr("ROS发布器未初始化")
            return False
        
        try:
            # 分离左右臂动作
            left_cmd, right_cmd = actions[:8], actions[8:]
            
            # 创建并发布AlohaCmd消息
            from sdk.msg import AlohaCmd
            aloha_cmd = AlohaCmd()
            aloha_cmd.arx_pos_left = left_cmd[:-1]
            aloha_cmd.arx_pos_right = right_cmd[:-1]
            aloha_cmd.cmd_left = 2
            aloha_cmd.cmd_right = 2
            self.aloha_cmd_pub.publish(aloha_cmd)
            
            # 创建并发布夹爪控制消息
            gripper_msg = JointState()
            gripper_msg.header.stamp = rospy.Time.now()
            gripper_msg.position = [
                50.0 if left_cmd[-1] > 35.0 else 0.0,
                50.0 if right_cmd[-1] > 35.0 else 0.0
            ]
            self.gripper_action_pub.publish(gripper_msg)
            
            # 控制发布频率
            if self.rate:
                self.rate.sleep()
            
            return True
        except Exception as e:
            rospy.logerr(f"发布动作指令时出错: {str(e)}")
            return False
    
    def run_inference(self, single_step=False):
        """执行推理并发布动作
        
        Args:
            single_step: 是否只执行一次推理，默认为False表示持续推理
            
        Returns:
            单步模式下返回是否成功，连续模式下无返回值
        """
        if not self.is_initialized:
            rospy.logerr("推理引擎未初始化")
            return False
        
        if not single_step:
            self.is_running = True
        
        try:
            while not rospy.is_shutdown() and (single_step or self.is_running):
                # 获取观测数据
                observation = self.get_observation()
                
                if observation:
                    # 处理观测数据
                    for name in observation:
                        if "image" in name:
                            # 图像归一化
                            observation[name] = observation[name].type(torch.float32) / 255
                            # 调整通道顺序
                            observation[name] = observation[name].permute(2, 0, 1).contiguous()
                        # 添加批次维度
                        observation[name] = observation[name].unsqueeze(0)
                        # 迁移到指定设备
                        observation[name] = observation[name].to(self.device)
                    
                    # 计时
                    start_time = time.time()
                    
                    # 执行推理
                    with torch.no_grad():
                        action = self.policy.select_action(observation)
                    
                    # 处理动作
                    action = action.squeeze(0)
                    action = action.to("cpu")
                    action_np = action.detach().numpy()
                    
                    # 发布动作
                    self.publish_actions(action_np.tolist())
                    
                    # 计算推理时间
                    inference_time = time.time() - start_time
                    rospy.loginfo(f"推理时间: {inference_time:.4f}秒")
                    
                    # 单步模式下返回
                    if single_step:
                        return True
                else:
                    rospy.logwarn("未获取到完整的观测数据")
                    if self.rate:
                        self.rate.sleep()
                    
                    if single_step:
                        return False
            
            # 如果循环退出，则重置运行状态
            self.is_running = False
            
        except Exception as e:
            rospy.logerr(f"执行推理时出错: {str(e)}")
            self.is_running = False
            return False
    
    def stop(self):
        """停止推理"""
        self.is_running = False 