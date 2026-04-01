import numpy as np
#Scientific Python (科学计算库)
import scipy.linalg as la

def compute_lqr_gain(A,B,Q,R):


    #Solve Continuous Algebraic Riccati Equation（求解连续代数黎卡提方程）
    P=la.solve_continuous_are(A,B,Q,R)

    """
    inverse求解逆矩阵 
    增益矩阵 K的计算公式是 K = R^{-1} B^T P。
    在 Python (NumPy) 中，矩阵乘法使用 @ 符号，转置使用 .T
    
    """
    
    K=np.linalg.inv(R)@ B.T @ P

    return K

