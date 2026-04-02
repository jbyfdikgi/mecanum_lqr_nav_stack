import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Float32
import numpy as np
# 导入写的数学引擎
from .lqr_math import compute_lqr_gain

class LQRControllerNode(Node):
    def __init__(self):
        super().__init__("lqr_node")
        #获取当前时间
        self.state_time=self.get_clock().now()
        #动力学参数（A,B） 要和物理法则一致
        self.A=np.array([[0.0,1.0],[0.0,-0.1]])
        self.B=np.array([[0.0],[1.0]])

        #LQR权重参数（Q,R） Q对误差的惩罚 R对能量消耗的惩罚
        self.Q=np.array([[10.0,0.0],[0.0,1.0]])
        self.R=np.array([[1.0]])

        #目标设定 最终希望 p=5.0 v=0.0
        self.target=np.array([[5.0],[0.0]])

        #物理环境不变，K 矩阵只需要算一次
        self.K = compute_lqr_gain(self.A, self.B, self.Q, self.R)
        self.get_logger().info(f'成功计算最优增益矩阵 K: {self.K}')

        #创建订阅者获取v p
        self.sub_state = self.create_subscription(
            Float32MultiArray, '/cart/state', self.state_callback, 10)    
        #创建发布者发布控制力 
        self.pub_u = self.create_publisher(Float32, '/lqr/control_input', 10)

    def state_callback(self,msg):
        #计算时间差
        now=self.get_clock().now()
        t=(now-self.state_time).nanoseconds /1e9

        #这里让目标点呈时间正弦变化，追踪动态目标点
        target_p=5.0*np.sin(0.5*t)
        target_v = 2.5 * np.cos(0.5 * t)
        self.target = np.array([[target_p], [target_v]])

        #处理v p数据
        current = np.array([[msg.data[0]], [msg.data[1]]])
        #计算误差 (Error = Current - Target)
        error = current - self.target
        #核心控制律计算：u = -K * error
        u_matrix = -self.K @ error       
        #发布控制力 (取矩阵里的第一个标量值)
        control_msg = Float32()
        control_msg.data = float(u_matrix[0, 0])
        self.pub_u.publish(control_msg)

def main(args=None):
    rclpy.init(args=args)
    node = LQRControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
