import rclpy
import math
import numpy as np
from rclpy.node import Node 

class ForwardKinematics():
    def __init__(self, lift_pos, arm_pos, wrist_pos):
        # Get joint values as node parameters that can be set from CLI
        self.a1 = 0
        self.alpha1 = 0
        self.d1 = 0.2
        self.theta1 = 0
        self.a2 = 0
        self.alpha2 = 0
        self.d2 = lift_pos
        self.theta2 = 0
        self.a3 = 0
        self.alpha3 = -1.5708
        self.d3 = 0.5 + arm_pos
        self.theta3 = -wrist_pos
        self.a4 = 0
        self.alpha4 = 1.5708
        self.d4 = 0
        self.theta4 = 1.5708
  
    def print_dh_table(self):
        # Fancy print of the DH table here
        return 0    

    def get_hm_matrix(self, a, alpha, d, theta):
        # Angles are in radians
        Ax = np.array([(math.cos(theta), -math.sin(theta)*math.cos(alpha), math.sin(theta)*math.sin(alpha), a*math.cos(theta)),
                       (math.sin(theta), math.cos(theta)*math.cos(alpha), -math.cos(theta)*math.sin(alpha), a*sin(theta)),
                       (0, math.sin(alpha), math.cos(alpha), d),
                       (0, 0, 0, 1)])
        return Ax

    def hm_matrices(self):
        # Create numpy array to deal with matrix operations
        A1 = get_hm_matrix(self.a1, self.alpha1, self.d1, self.theta1)
        A2 = get_hm_matrix(self.a2, self.alpha2, self.d2, self.theta2)
        A3 = get_hm_matrix(self.a3, self.alpha3, self.d3, self.theta3)
        A4 = get_hm_matrix(self.a4, self.alpha4, self.d4, self.theta4)

    def compute_fk(self):
        An = A1*A2*A3*A4
        return 0

    def main(self):
        self.print_dh_table()
        
        return 0

if __name__ == '__main__':
    fk = ForwardKinematics(lift_pos=0, arm_pos=0, wrist_pos=0)
    fk.main()
