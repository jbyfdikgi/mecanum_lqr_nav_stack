import numpy as np
#Scientific Python (科学计算库)
import scipy.linalg as la
import math

class LqrMathEngine:
    def __init__(self):
        #Q R对角矩阵
        self.Q=np.diag([10.0,10.0,10.0])
        self.R=np.diag([0.1,0.1,0.1])

    def get_linearized_dynamics(self,theta,v_x,v_y):
        #计算动态A B矩阵
        A=np.zeros((3,3))
        #对theta求偏导的结果
        A[0, 2] = -v_x * math.sin(theta) - v_y * math.cos(theta)
        A[1, 2] =  v_x * math.cos(theta) - v_y * math.sin(theta)        

        B = np.zeros((3, 3))
        # 这是方程对 vx, vy, omega 求偏导
        B[0, 0] = math.cos(theta)
        B[0, 1] = -math.sin(theta)
        B[1, 0] = math.sin(theta)
        B[1, 1] = math.cos(theta)
        B[2, 2] = 1.0
        
        return A, B
    
    def solve_lqr(self, A, B):
        #使用scipy，求解连续黎卡提方程，返回K矩阵
        P = la.solve_continuous_are(A, B, self.Q, self.R)
        K = np.linalg.inv(self.R) @ B.T @ P
        return K

