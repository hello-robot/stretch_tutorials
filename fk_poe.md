In this tutorial, we will study the forward kinematics of Stretch. Technically, kinematics is the study of motion of objects without references to the forces which cause the motion. For Stretch or any other robot, the objects are the joints and links of the robot. Our intention is to study how these objects move with respect to each other and how changing one or more variables in the kinematic chain affects the other variables. To understand this, we derive the mathematical equations that define the motion of these frames.

The two most important frames in a robot are its base_link and ee_frame (ee short for end-effector). For Stretch, the base_link is defined squarely at the center of the wheels with axes pointing as shown in figure 1. The link_grasp_center (ee_frame) is defined as the center of the distal extremities of Stretch’s fingers as shown in figure 2.

Insert figure 1 with base_link

Insert figure 2 with link_grasp_center

Often, we need to obtain the end effector pose (task space) with respect to the base_link given some joint values (joint space) or vice-versa. The kinematic equations allow us to make these calculations.

Remember:
The forward kinematics allows us to convert from joint space to task space
The inverse kinematics allows us to convert from task space to joint space

We are going to use the Product of Exponentials method to derive the forward kinematics of Stretch. The steps are as outlined below:

(All matrix operations are denoted in MATLAB notation for convenience)

Step 1. Assign coordinate frames for all links of interest. Fortunately, the TF tree allows us to visualize the frames that have already been assigned to various links on Stretch. The links of interest are: base_link, link_lift, link_arm_l0, link_wrist_yaw, link_grasp_center (ee_frame). Go ahead and visualize them in RViz.

Step 2. Find the homogeneous transformation matrix for the link_grasp_center frame with respect to the base_link frame in the robot home configuration, when all joints are at their zero value. Let’s call this matrix M.

M = [R, p; 0, 1]
Where R is the 3x3 rotational matrix defined as,
R = [0, 1, 0; -1, 0, 0; 0, 0, 1] 
and p is the position vector of the ee_frame w.r.t. the base_link defined as,
p = [x; y; z]

Step 3. Calculate the twists for all joints: 
(Refer to book for an explanation on how the twists were derived)
S0 = [0, 0, 1, 0, 0, 0]’
S1 = [0, 0, 0, 0, 0, 1]’
S2 = [0, 0, 0, 0, -1, 0]’
S3 = [0, 0, 1, x1, y1, z1]’ 
where x1, y1, z1 are obtained by computing the cross-product of w3 and -(position of link_wrist_yaw)

Step 4. Use Rodriguez’s formula to obtain the transformation matrices for intermediate links:
Figure 3

Insert figure 3 to illustrate Rodriguez's formula

Step 5. Use Product of Exponentials formula to compute the forward kinematics
Figure 4

Insert figure 4 to illustrate PoE formula


