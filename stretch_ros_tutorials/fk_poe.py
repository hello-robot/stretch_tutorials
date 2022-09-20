import rclpy
import math
import numpy as np
from rclpy.node import Node
from math import sin as s, cos as c


class ForwardKinematics():
    def __init__(self, joint_lift, joint_arm, joint_wrist_yaw):
        # Get joint values as node parameters that can be set from CLI
        self.joint_lift = joint_lift
        self.joint_arm = joint_arm # joint_arm is the addition of joint_arm_l0 to l3
        self.joint_wrist_yaw = joint_wrist_yaw
  
    # Home configuration of the robot is when all the joint values are zero
    def get_home_configuration(self):
        # Rotation of link_grasp_center w.r.t. base_link
        # Assumed to be ideal for each robot but might have minor calibration errors
        R = np.array([[0, 1, 0], [-1, 0, 0], [0, 0, 1]])

        # Position of link_grasp_center w.r.t. base_link
        # should be fetched for each robot to account for minor calibration errors
        p = np.array([[-0.0107], [-0.3445], [0.0205]])

        M = np.block([[R, p], [0, 0, 0, 1]])
        return M

    def get_screw_axes(self):
        S1 = np.array([0, 0, 1, 0, 0, 0])
        S2 = np.array([0, 0, 0, 0, 0, 1])
        S3 = np.array([0, 0, 0, 0, -1, 0])
        S4 = np.array([0, 0, -1, 0.1406, -0.0173, 0]) # Must be fetched for each robot
        
        S = np.block([[S1], [S2], [S3], [S4]])
        return np.transpose(S)

    def get_transformation_matrix(self):
        l1 = self.joint_lift
        l2 = self.joint_arm
        t3 = -self.joint_wrist_yaw
        E1 = np.array([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, l1], [0, 0, 0, 1]])
        E2 = np.array([[1, 0, 0, 0], [0, 1, 0, -l2], [0, 0, 1, 0], [0, 0, 0, 1]])
        Y = np.array([[s(t3), 1-c(t3), 0], [c(t3)-1, s(t3), 0], [0, 0, t3]])
        V = np.array([[0.1406], [-0.0173], [0]])
        RE3 = np.array([[c(t3), s(t3), 0], [-s(t3), c(t3), 0], [0, 0, 1]])
        E3 = np.block([[RE3, np.matmul(Y, V)], [0, 0, 0, 1]])
        E12 = np.matmul(E1, E2)

        return np.matmul(E12, E3)

    def compute_fk(self):
        M = self.get_home_configuration()
        E = self.get_transformation_matrix()

        return np.matmul(E, M)


def main():
    print("Enter the joint values for joint_lift, joint_arm, joint_wrist_yaw: 0.6, 0.45, 3.145 for example")
    x, y, z = input("Enter here: ").split(',', 3)
    joint_lift, joint_arm, joint_wrist_yaw = float(x), float(y), float(z)
    fk = ForwardKinematics(joint_lift, joint_arm, joint_wrist_yaw)
    result = fk.compute_fk()
    print(result)


if __name__ == '__main__':
    main()



# import numpy as np
# from math import sin as s, cos as c

# R = np.array([[0, 1, 0], [-1, 0, 0], [0, 0, 1]])

# # Position of link_grasp_center w.r.t. base_link
# # should be fetched for each robot to account for minor calibration errors
# p = np.array([[-0.0107], [-0.3445], [0.0205]])

# M = np.block([[R, p], [0, 0, 0, 1]])

# t3 = -1.33
# l1 = 0.5003
# l2 = 0.2025
# E1 = np.array([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, l1], [0, 0, 0, 1]])
# E2 = np.array([[1, 0, 0, 0], [0, 1, 0, -l2], [0, 0, 1, 0], [0, 0, 0, 1]])
# Y = np.array([[s(t3), 1-c(t3), 0], [c(t3)-1, s(t3), 0], [0, 0, t3]])
# V = np.array([[-0.1406], [0.0173], [0]])
# RE3 = np.array([[c(t3), s(t3), 0], [-s(t3), c(t3), 0], [0, 0, 1]])
# E3 = np.block([[RE3, np.matmul(Y, V)], [0, 0, 0, 1]])

# E12 = np.matmul(E1, E2)
# E3M = np.matmul(E3, M)
# print(np.matmul(E12, E3M))
