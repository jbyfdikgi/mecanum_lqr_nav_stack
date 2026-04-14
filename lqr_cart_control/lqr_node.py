import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist
import numpy as np
from lqr_cart_control.lqr_math import LqrMathEngine
import math


class LqrMecanumController(Node):
    def __init__(self):
        super().__init__("lqr_mecanum_controller")
        #创建订阅发布者
        self.sub_state = self.create_subscription(Float32MultiArray, '/cart/state_2d', self.state_callback, 10)
        self.pub_cmd = self.create_publisher(Twist, 'cmd_vel', 10)
        self.pub_target = self.create_publisher(Float32MultiArray, '/cart/target_2d', 10)
        self.math_engine = LqrMathEngine()
        self.current_state = np.zeros((3, 1)) 
        self.last_u = np.zeros((3, 1))        
        self.start_time = self.get_clock().now()
        #创建定时器进行发布
        self.timer = self.create_timer(0.01, self.control_loop)
        self.get_logger().info('Initialization successful')

    def state_callback(self,msg):
        #更新数据
        self.current_state[0, 0] = msg.data[0] 
        self.current_state[1, 0] = msg.data[1] 
        self.current_state[2, 0] = msg.data[2]

    def control_loop(self):
        t = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        #生成8字形目标
        target_x = 3.0 * math.sin(0.5 * t)
        target_y = 3.0 * math.sin(1.0 * t)
        target_theta = 0.0  
        v_target_global_x = 1.5 * math.cos(0.5 * t)
        v_target_global_y = 3.0 * math.cos(1.0 * t)
        #发布目标位置
        target_msg = Float32MultiArray()
        target_msg.data = [target_x, target_y, target_theta]
        self.pub_target.publish(target_msg)   
        # --- 步骤 2：前馈控制 (逆投影) ---
        theta = float(self.current_state[2, 0])
        u_ff_x = v_target_global_x * math.cos(theta) + v_target_global_y * math.sin(theta)
        u_ff_y = -v_target_global_x * math.sin(theta) + v_target_global_y * math.cos(theta)
        u_ff_omega = 0.0 
        u_ff = np.array([[u_ff_x], [u_ff_y], [u_ff_omega]]) 
        
        #LQR 反馈控制
        e_x = self.current_state[0, 0] - target_x
        e_y = self.current_state[1, 0] - target_y
        e_theta = self.current_state[2, 0] - target_theta
        e_theta = math.atan2(math.sin(e_theta), math.cos(e_theta))
        error = np.array([[e_x], [e_y], [e_theta]])
        
        last_vx = float(self.last_u[0, 0])
        last_vy = float(self.last_u[1, 0])
        
        A, B = self.math_engine.get_linearized_dynamics(theta, last_vx, last_vy)
        K = self.math_engine.solve_lqr(A, B)
        u_fb = -K @ error
        
        #指令下发
        u_total = u_ff + u_fb
        self.last_u = u_total 
        
        cmd_msg = Twist()
        cmd_msg.linear.x = float(u_total[0, 0])
        cmd_msg.linear.y = float(u_total[1, 0])
        cmd_msg.angular.z = float(u_total[2, 0])
        self.pub_cmd.publish(cmd_msg)



def main(args=None):
    rclpy.init(args=args)
    node = LqrMecanumController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
