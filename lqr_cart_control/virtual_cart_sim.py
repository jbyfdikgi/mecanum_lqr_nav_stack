import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist
import math

class VirtualCartSim(Node):
    def __init__(self):
        super().__init__("virtual_cart_sim")
        #初始物理状态
        self.x=0.0
        self.y=0.0
        self.theta=0.0 #弧度制
        #输入控制初始化
        self.v_x=0.0
        self.v_y=0.0
        self.omega=0.0

        #订阅cmd_vel
        self.sub_cmd=self.create_subscription(Twist,"cmd_vel",self.cmd_callback,10)
        
        #发布小车状态
        self.pub_odom=self.create_publisher(Float32MultiArray,"/cart/state_2d",10)

        #定时计算小车物理状态
        self.dt=0.01
        self.timer=self.create_timer(self.dt,self.physics_step)

        self.get_logger().info("虚拟小车仿真启动")

    def cmd_callback(self,msg):
        #从Twist信息中获取三个控制自由度
        self.v_x = msg.linear.x
        self.v_y = msg.linear.y
        self.omega = msg.angular.z
        
        

    def physics_step(self):       
        #计算在当前偏航角theta下，局部速度在全局X,Y下的投影
        dx=self.v_x*math.cos(self.theta)-self.v_y*math.sin(self.theta)
        dy=self.v_x*math.sin(self.theta)+self.v_y*math.cos(self.theta)
        dtheta = self.omega

        #数值积分 
        self.x += dx * self.dt
        self.y += dy * self.dt
        self.theta += dtheta * self.dt

        # 将角度规范化到-pi到pi 
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))

        #发布状态
        state_msg = Float32MultiArray()
        state_msg.data = [self.x, self.y, self.theta]
        self.pub_odom.publish(state_msg)


def main(args=None):
    rclpy.init(args=args)
    node=VirtualCartSim()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()    

