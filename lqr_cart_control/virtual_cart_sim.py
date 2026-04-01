import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray , Float32
import random


class VirtualCartSim(Node):
    def __init__(self):
        super().__init__("virtual_cart_sim")
        #物理状态初始化 position velocity
        self.p=0.0
        self.v=0.0
        #控制输入初始化 推力
        self.u=0.0
        #创建订阅者 代替cmd_vel
        self.sub_control=self.create_subscription(Float32,"/lqr/control_input",self.control_callback,10)
        #发布者 代替odom
        self.pub_state=self.create_publisher(Float32MultiArray,'/cart/state', 10)
        # 创建Timer
        self.dt=0.01
        self.timer=self.create_timer(self.dt,self.math_cal)

        
    def control_callback(self,msg):
        #根据反馈修改力控制
        self.u=msg.data

    def math_cal(self):
        #牛顿第二定律算加速度 m=1 风阻为0.1倍的v
        a = self.u - 0.1 * self.v
        #欧拉法数值积分
        self.v += a * self.dt
        self.p += self.v * self.dt
        #加入高斯噪音模拟现实编码器误差（根据中心极限定理，多个误差累加最终会显示出正态分布特性）
        noisy_p=self.p+random.gauss(0.0,0.05) #填入均值 方差
        noisy_v=self.v+random.gauss(0.0,0.05) #填入均值 方差
        #通过'/cart/state'发布小车虚拟状态
        state_msg=Float32MultiArray()
        state_msg.data=[noisy_p,noisy_v]
        self.pub_state.publish(state_msg)
        

def main(args=None):
    rclpy.init(args=args)
    node=VirtualCartSim()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()    

