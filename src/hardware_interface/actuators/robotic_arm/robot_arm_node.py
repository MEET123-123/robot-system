import rtde_control
import rtde_receive
import numpy as np
import threading
import time
from enum import Enum


class RobotArm(Node):
    """手术机器人机械臂控制节点"""

    class ArmState(Enum):
        IDLE = 0
        MOVING = 1
        PAUSED = 2
        ERROR = 3

    def __init__(self):
        super().__init__('robot_arm')

        # 状态变量
        self.state = self.ArmState.IDLE
        self.current_joint_positions = []
        self.current_pose = None
        self.joint_names = [
            'joint_1', 'joint_2', 'joint_3',
            'joint_4', 'joint_5', 'joint_6'
        ]

        # TF监听器
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # 创建动作客户端
        self.trajectory_client = ActionClient(
            self,
            FollowJointTrajectory,
            'arm_controller/follow_joint_trajectory'
        )

        # 创建服务
        self.start_srv = self.create_service(
            Trigger,
            'arm/start',
            self.start_callback
        )

        self.stop_srv = self.create_service(
            Trigger,
            'arm/stop',
            self.stop_callback
        )

        self.pause_srv = self.create_service(
            Trigger,
            'arm/pause',
            self.pause_callback
        )

        self.resume_srv = self.create_service(
            Trigger,
            'arm/resume',
            self.resume_callback
        )

        self.homing_srv = self.create_service(
            Trigger,
            'arm/homing',
            self.homing_callback
        )

        self.set_velocity_srv = self.create_service(
            SetBool,
            'arm/set_velocity_scaling',
            self.set_velocity_callback
        )

        # 创建订阅者
        self.joint_state_sub = self.create_subscription(
            JointState,
            'joint_states',
            self.joint_state_callback,
            10
        )

        # 创建发布者
        self.cartesian_velocity_pub = self.create_publisher(
            TwistStamped,
            'arm/cartesian_velocity_command',
            10
        )

        self.get_logger().info("机械臂节点已启动")

        # 等待动作服务器准备好
        self.get_logger().info("等待轨迹动作服务器...")
        self.trajectory_client.wait_for_server()
        self.get_logger().info("轨迹动作服务器已准备好")

    def joint_state_callback(self, msg):
        """关节状态回调函数"""
        # 更新当前关节位置
        self.current_joint_positions = []
        for name in self.joint_names:
            if name in msg.name:
                idx = msg.name.index(name)
                self.current_joint_positions.append(msg.position[idx])

        # 尝试获取末端执行器位姿
        try:
            transform = self.tf_buffer.lookup_transform(
                'base_link',
                'tool0',
                rclpy.time.Time()
            )
            self.current_pose = transform
        except TransformException as ex:
            self.get_logger().debug(f"无法获取位姿: {ex}")

    def start_callback(self, request, response):
        """启动机械臂服务回调"""
        if self.state == self.ArmState.IDLE or self.state == self.ArmState.PAUSED:
            self.state = self.ArmState.IDLE
            response.success = True
            response.message = "机械臂已启动"
            self.get_logger().info("机械臂已启动")
        else:
            response.success = False
            response.message = f"机械臂状态错误: {self.state.name}"
            self.get_logger().warn(f"无法启动机械臂，当前状态: {self.state.name}")

        return response

    def stop_callback(self, request, response):
        """停止机械臂服务回调"""
        if self.state == self.ArmState.MOVING or self.state == self.ArmState.PAUSED:
            # 发送取消请求
            self.trajectory_client.cancel_all_goals()
            self.state = self.ArmState.IDLE
            response.success = True
            response.message = "机械臂已停止"
            self.get_logger().info("机械臂已停止")
        else:
            response.success = False
            response.message = f"机械臂状态错误: {self.state.name}"
            self.get_logger().warn(f"无法停止机械臂，当前状态: {self.state.name}")

        return response

    def pause_callback(self, request, response):
        """暂停机械臂服务回调"""
        if self.state == self.ArmState.MOVING:
            # 发送取消请求
            self.trajectory_client.cancel_all_goals()
            self.state = self.ArmState.PAUSED
            response.success = True
            response.message = "机械臂已暂停"
            self.get_logger().info("机械臂已暂停")
        else:
            response.success = False
            response.message = f"机械臂状态错误: {self.state.name}"
            self.get_logger().warn(f"无法暂停机械臂，当前状态: {self.state.name}")

        return response

    def resume_callback(self, request, response):
        """恢复机械臂服务回调"""
        if self.state == self.ArmState.PAUSED:
            # 这里应该实现继续执行之前的轨迹
            # 简化实现，实际应记录并恢复轨迹
            response.success = False
            response.message = "恢复功能未实现"
            self.get_logger().warn("恢复功能未实现")
        else:
            response.success = False
            response.message = f"机械臂状态错误: {self.state.name}"
            self.get_logger().warn(f"无法恢复机械臂，当前状态: {self.state.name}")

        return response

    def homing_callback(self, request, response):
        """机械臂回零服务回调"""
        if self.state == self.ArmState.IDLE or self.state == self.ArmState.PAUSED:
            # 创建归位轨迹
            home_positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            success = self.move_to_joint_positions(home_positions, duration=5.0)

            if success:
                response.success = True
                response.message = "机械臂正在回零"
                self.get_logger().info("机械臂正在回零")
            else:
                response.success = False
                response.message = "发送回零命令失败"
                self.get_logger().error("发送回零命令失败")
        else:
            response.success = False
            response.message = f"机械臂状态错误: {self.state.name}"
            self.get_logger().warn(f"无法回零，当前状态: {self.state.name}")

        return response

    def set_velocity_callback(self, request, response):
        """设置速度缩放服务回调"""
        # 实际应用中应实现速度缩放功能
        # 简化实现，仅记录请求
        self.get_logger().info(f"设置速度缩放: {request.data}")
        response.success = True
        response.message = f"速度缩放已设置为: {request.data}"
        return response

    def move_to_joint_positions(self, positions, duration=2.0):
        """移动机械臂到指定关节位置

        Args:
            positions: 目标关节位置列表
            duration: 运动持续时间(秒)

        Returns:
            bool: 是否成功发送运动命令
        """
        if len(positions) != len(self.joint_names):
            self.get_logger().error(f"关节位置数量不匹配: {len(positions)} != {len(self.joint_names)}")
            return False

        if self.state != self.ArmState.IDLE and self.state != self.ArmState.PAUSED:
            self.get_logger().error(f"机械臂状态错误: {self.state.name}")
            return False

        # 创建轨迹点
        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start = Duration(sec=int(duration), nanosec=int((duration % 1) * 1e9))

        # 创建轨迹
        trajectory = JointTrajectory()
        trajectory.joint_names = self.joint_names
        trajectory.points.append(point)

        # 创建目标
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory = trajectory
        goal_msg.goal_time_tolerance = Duration(sec=1)

        # 发送目标
        self.state = self.ArmState.MOVING
        self._send_trajectory_goal(goal_msg)

        return True

    def move_to_pose(self, pose, duration=2.0):
        """移动机械臂到指定位姿(在基坐标系下)

        Args:
            pose: 目标位姿 [x, y, z, rx, ry, rz] (单位: 米, 弧度)
            duration: 运动持续时间(秒)

        Returns:
            bool: 是否成功发送运动命令
        """
        if self.state != self.ArmState.IDLE and self.state != self.ArmState.PAUSED:
            self.get_logger().error(f"机械臂状态错误: {self.state.name}")
            return False

        # 这里应该实现逆运动学计算
        # 简化实现，仅打印目标位姿
        self.get_logger().info(f"移动到目标位姿: {pose}")

        # 实际应用中应调用逆运动学库计算关节角度
        # 这里使用模拟关节角度
        joint_positions = [
            pose[0] * 0.5,  # 简化的映射，实际应使用逆运动学
            pose[1] * 0.5,
            pose[2] * 0.5,
            pose[3] * 0.5,
            pose[4] * 0.5,
            pose[5] * 0.5
        ]

        return self.move_to_joint_positions(joint_positions, duration)

    def move_cartesian(self, velocity, duration=0.1):
        """笛卡尔空间运动

        Args:
            velocity: 线速度和角速度 [vx, vy, vz, wx, wy, wz] (单位: 米/秒, 弧度/秒)
            duration: 发送命令的持续时间(秒)
        """
        # 创建Twist消息
        twist_msg = TwistStamped()
        twist_msg.header.stamp = self.get_clock().now().to_msg()
        twist_msg.header.frame_id = 'base_link'

        twist_msg.twist.linear.x = velocity[0]
        twist_msg.twist.linear.y = velocity[1]
        twist_msg.twist.linear.z = velocity[2]
        twist_msg.twist.angular.x = velocity[3]
        twist_msg.twist.angular.y = velocity[4]
        twist_msg.twist.angular.z = velocity[5]

        # 发布消息
        self.cartesian_velocity_pub.publish(twist_msg)

    def _send_trajectory_goal(self, goal_msg):
        """发送轨迹目标"""

        def goal_response_callback(future):
            goal_handle = future.result()
            if not goal_handle.accepted:
                self.get_logger().error('目标被拒绝')
                self.state = self.ArmState.IDLE
                return

            self.get_logger().info('目标已接受')

            # 获取结果
            def get_result_callback(future):
                result = future.result().result
                status = future.result().status
                if status == 2:  # SUCCEEDED
                    self.get_logger().info('轨迹执行成功')
                    self.state = self.ArmState.IDLE
                else:
                    self.get_logger().error(f'轨迹执行失败，状态: {status}')
                    self.state = self.ArmState.ERROR

            goal_handle.get_result_async().add_done_callback(get_result_callback)

        # 发送目标异步
        self.trajectory_client.send_goal_async(
            goal_msg,
            feedback_callback=self._feedback_callback
        ).add_done_callback(goal_response_callback)

    def _feedback_callback(self, feedback_msg):
        """轨迹执行反馈回调"""
        # 可以在这里更新实时状态
        pass

    def get_status(self):
        """获取机械臂状态"""
        return {
            'state': self.state.name,
            'joint_positions': self.current_joint_positions,
            'current_pose': self.current_pose
        }


def main(args=None):
    rclpy.init(args=args)
    node = RobotArm()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()