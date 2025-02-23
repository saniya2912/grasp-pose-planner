import numpy as np

from leap_hand_utils.dynamixel_client import *
import leap_hand_utils.leap_hand_utils as lhu
import time
import mujoco
#######################################################
"""This can control and query the LEAP Hand

I recommend you only query when necessary and below 90 samples a second.  Each of position, velociy and current costs one sample, so you can sample all three at 30 hz or one at 90hz.

#Allegro hand conventions:
#0.0 is the all the way out beginning pose, and it goes positive as the fingers close more and more
#http://wiki.wonikrobotics.com/AllegroHandWiki/index.php/Joint_Zeros_and_Directions_Setup_Guide I belive the black and white figure (not blue motors) is the zero position, and the + is the correct way around.  LEAP Hand in my videos start at zero position and that looks like that figure.

#LEAP hand conventions:
#180 is flat out for the index, middle, ring, fingers, and positive is closing more and more.

"""
class init_blah:
    def __init__(self):
    #You can put the correct port here or have the node auto-search for a hand at the first 3 ports.
        self.motors = motors = [0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15]
        try:
            self.dxl_client = DynamixelClient(motors, '/dev/ttyUSB0', 4000000)
            self.dxl_client.connect()
        except Exception as e:
            print("[DEBUG]", e)
            try:
                self.dxl_client = DynamixelClient(motors, '/dev/ttyUSB1', 4000000)
                self.dxl_client.connect()
            except Exception:
                self.dxl_client = DynamixelClient(motors, '/dev/ttyUSB2', 4000000)
                self.dxl_client.connect()
########################################################
    def read_pos(self):
        return self.dxl_client.read_pos()
    
    def read_pos_leap(self):
        pos=self.dxl_client.read_pos()-(np.ones(16)*3.14)
        return pos
    #read velocity
    def read_vel(self):
        return self.dxl_client.read_vel()
    #read current
    def read_cur(self):
        return self.dxl_client.read_cur()

init_ins = init_blah()

class LeapNode_Poscontrol:
    def __init__(self):
        ####Some parameters
        # self.ema_amount = float(rospy.get_param('/leaphand_node/ema', '1.0')) #take only current
        self.kP = 500
        self.kI = 0
        self.kD = 25
        self.kP_slow = 300
        self.kI = 0
        self.kD = 200
        self.curr_lim = 1000
        self.prev_pos = self.pos = self.curr_pos = lhu.allegro_to_LEAPhand(np.zeros(16))
        self.motors = motors = [0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15]
        self.dxl_client=init_ins.dxl_client

        self.dxl_client.set_torque_enabled(self.motors, False)
        ADDR_SET_MODE = 11
        LEN_SET_MODE = 1
        self.dxl_client.sync_write(self.motors, np.ones(len(self.motors)) * 3, ADDR_SET_MODE, LEN_SET_MODE)
        self.dxl_client.set_torque_enabled(self.motors, True)

        #Enables position-current control mode and the default parameters, it commands a position and then caps the current so the motors don't overload
        self.dxl_client.sync_write(motors, np.ones(len(motors))*5, 11, 1)
        self.dxl_client.set_torque_enabled(motors, True)
        self.dxl_client.sync_write(motors, np.ones(len(motors)) * self.kP, 84, 2) # Pgain stiffness     
        self.dxl_client.sync_write([0,4,8], np.ones(3) * (self.kP * 0.75), 84, 2) # Pgain stiffness for side to side should be a bit less
        self.dxl_client.sync_write(motors, np.ones(len(motors)) * self.kI, 82, 2) # Igain
        self.dxl_client.sync_write(motors, np.ones(len(motors)) * self.kD, 80, 2) # Dgain damping
        self.dxl_client.sync_write([0,4,8], np.ones(3) * (self.kD * 0.75), 80, 2) # Dgain damping for side to side should be a bit less
        #Max at current (in unit 1ma) so don't overheat and grip too hard #500 normal or #350 for lite
        self.dxl_client.sync_write(motors, np.ones(len(motors)) * self.curr_lim, 102, 2)
        #self.dxl_client.write_desired_pos(self.motors, self.curr_pos)

    #Receive LEAP pose and directly control the robot
    def set_leap(self, pose):
        self.prev_pos = self.curr_pos
        self.curr_pos = np.array(pose)
        self.dxl_client.write_desired_pos(self.motors, self.curr_pos)
    #allegro compatibility
    def set_allegro(self, pose):
        pose = lhu.allegro_to_LEAPhand(pose, zeros=False)
        self.prev_pos = self.curr_pos
        self.curr_pos = np.array(pose)
        self.dxl_client.write_desired_pos(self.motors, self.curr_pos)
    #Sim compatibility, first read the sim value in range [-1,1] and then convert to leap
    def set_ones(self, pose):
        pose = lhu.sim_ones_to_LEAPhand(np.array(pose))
        self.prev_pos = self.curr_pos
        self.curr_pos = np.array(pose)
        self.dxl_client.write_desired_pos(self.motors, self.curr_pos)
    #read position
    def read_pos(self):
        return self.dxl_client.read_pos()
    
    def read_pos_leap(self):
        pos=self.dxl_client.read_pos()-(np.ones(16)*3.14)
        return pos
    #read velocity
    def read_vel(self):
        return self.dxl_client.read_vel()
    #read current
    def read_cur(self):
        return self.dxl_client.read_cur()
    
    def cubic_trajectory(self,q0, v0, q1, v1, t0, t1, current_time):
        # Ensure the current time is within the valid time range
        current_time = np.clip(current_time, t0, t1)

        # Define the matrix M for scalar time t0 and t1 (applies to all elements)
        M = np.array([
            [1, t0, t0**2, t0**3],
            [0, 1, 2*t0, 3*t0**2],
            [1, t1, t1**2, t1**3],
            [0, 1, 2*t1, 3*t1**2]
        ])

        # Stack the q0, v0, q1, v1 values into a matrix (each as a 16-element array)
        b = np.vstack([q0, v0, q1, v1])

        # Solve for the coefficients a for each set of q0, v0, q1, v1
        a = np.linalg.inv(M).dot(b)

        # Compute position (qd), velocity (vd), and acceleration (ad) for each element
        qd = a[0] + a[1]*current_time + a[2]*current_time**2 + a[3]*current_time**3
        vd = a[1] + 2*a[2]*current_time + 3*a[3]*current_time**2
        ad = 2*a[2] + 6*a[3]*current_time

        return qd
    

class LeapNode_Taucontrol:
    def __init__(self):
    # List of motor IDs
        self.motors = [0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15]
        self.dxl_client=init_ins.dxl_client

        
        self.dxl_client.set_torque_enabled(self.motors, False)
        # Set the control mode to Torque Control Mode
        # Address 11 typically corresponds to setting the operating mode, and 0 is Torque Control Mode
        ADDR_SET_MODE = 11
        LEN_SET_MODE = 1
        self.dxl_client.sync_write(self.motors, np.ones(len(self.motors))* 0, ADDR_SET_MODE, LEN_SET_MODE)
        self.dxl_client.set_torque_enabled(self.motors, True)

        # Set the current limit for Torque Control (Goal Current)
        # Address 102 might correspond to Goal Current, and 2 bytes is the length
        ADDR_GOAL_CURRENT = 102
        LEN_GOAL_CURRENT = 2
        self.curr_lim = 350  # Adjust the current limit as needed (this is just an example)
        self.dxl_client.sync_write(self.motors, np.ones(len(self.motors)) * self.curr_lim, ADDR_GOAL_CURRENT, LEN_GOAL_CURRENT)

    def set_desired_torque(self, desired_torque):
    # Convert desired torque to the corresponding current (depends on the motor's torque constant)
    # For example, assume a torque constant where 1 unit of torque corresponds to 1 unit of current
        

        # Ensure all elements in the desired_torque are scalars
        # For instance, you can flatten the list if necessary
        desired_torque_flat = [float(torque) for torque in desired_torque]  # Convert all to floats

        # Convert to NumPy array and calculate the current
        desired_current = np.array([torque / 0.51 for torque in desired_torque_flat])
        # Adjust this based on your motor's torque constant

        # Address for the Goal Current (or Torque) register
        ADDR_GOAL_CURRENT = 102
        LEN_GOAL_CURRENT = 2  # Length is usually 2 bytes

        # Write the desired current (which corresponds to the desired torque) to all motors
        self.dxl_client.sync_write(self.motors, desired_current*1000, ADDR_GOAL_CURRENT, LEN_GOAL_CURRENT)

    def read_pos(self):
        return self.dxl_client.read_pos()
    
    def read_pos_leap(self):
        pos=self.dxl_client.read_pos()-(np.ones(16)*3.14)
        return pos
    #read velocity
    def read_vel(self):
        return self.dxl_client.read_vel()
    #read current
    def read_cur(self):
        return self.dxl_client.read_cur()

        
import mujoco
import numpy as np

class OnlyPosIK:
    def __init__(self,xml_path):
        self.model = mujoco.MjModel.from_xml_path(xml_path)
        self.data = mujoco.MjData(self.model)
        self.jacp = np.zeros((3, self.model.nv))  # translation jacobian
        self.jacr = np.zeros((3, self.model.nv)) 
        self.step_size = 0.5
        self.tol = 0.01
        self.alpha = 0.5
        self.init_q = [0.0, 0.0, 0.0, 0.0]  
    
    def check_joint_limits(self, q):
        """Check if the joints is under or above its limits"""
        for i in range(len(q)):
            q[i] = max(self.model.jnt_range[i][0], min(q[i], self.model.jnt_range[i][1]))

    #Gradient Descent pseudocode implementation
    def calculate(self, goal, site_name):
        site_id=self.model.site(site_name).id
        self.data.qpos = self.init_q
        mujoco.mj_forward(self.model, self.data)
        current_pose = self.data.site(site_id).xpos
        error = np.subtract(goal, current_pose)

        max_iterations = 100000
        iteration = 0

        while (np.linalg.norm(error) >= self.tol) and iteration < max_iterations:
            #calculate jacobian
            mujoco.mj_jacSite(self.model, self.data, self.jacp, self.jacr,site_id)
            #calculate gradient
            grad = self.alpha * self.jacp.T @ error
            #compute next step
            self.data.qpos += self.step_size * grad
            #check joint limits
            self.check_joint_limits(self.data.qpos)
            #compute forward kinematics
            mujoco.mj_forward(self.model, self.data) 
            #calculate new error
            error = np.subtract(goal, self.data.site(site_id).xpos)

            iteration += 1

        if iteration >= max_iterations:
            print("Warning: Maximum iterations reached. The solution may not have converged.")
        
        result = self.data.qpos.copy()
        return result
    
    def calc_jac(self,site_name):
        J1=mujoco.mj_JacSite()
    
class GraspPose:
    def __init__(self):
        pass

    def find_finger(self,site_name1,site_name2):
        finger_mapping = {
            '1_1': 'index', '1_2': 'index', '1_3': 'index', '1_4': 'index',
            '2_1': 'middle', '2_2': 'middle', '2_3': 'middle', '2_4': 'middle',
            '3_1': 'ring', '3_2': 'ring', '3_3': 'ring', '3_4': 'ring',
            '4_1': 'thumb', '4_2': 'thumb', '4_3': 'thumb'
        }

        finger1_name = finger_mapping.get(site_name1, None)
        finger2_name = finger_mapping.get(site_name2, None)

        return finger1_name, finger2_name

    def ik_name(self, site_name1, site_name2):
        # Use find_finger to get the finger names
        finger1_name, finger2_name = self.find_finger(site_name1, site_name2)
        
        # Map the finger names to their corresponding IK names
        ik_mapping = {
            'index': 'indexik',
            'middle': 'middleik',
            'ring': 'ringik',
            'thumb': 'thumbik'
        }

        # Get the IK names for each finger
        finger1_ik = ik_mapping.get(finger1_name, None)
        finger2_ik = ik_mapping.get(finger2_name, None)

        return finger1_ik, finger2_ik
    
    def ik_name(self, site_name1, site_name2):
        # Use find_finger to get the finger names
        finger1_name, finger2_name = self.find_finger(site_name1, site_name2)
        
        # Map the finger names to their corresponding IK names
        ik_mapping = {
            'index': 'indexik',
            'middle': 'middleik',
            'ring': 'ringik',
            'thumb': 'thumbik'
        }

        # Get the IK names for each finger
        finger1_ik = ik_mapping.get(finger1_name, None)
        finger2_ik = ik_mapping.get(finger2_name, None)

        return finger1_ik, finger2_ik
    
    def joint_angles(self,site_name1, site_name2,q1,q2):

        q_index = [0, 0, 0, 0]
        q_middle = [0, 0, 0, 0]
        q_ring = [0, 0, 0, 0]
        q_thumb = [0, 0, 0, 0]

        # Identify the fingers based on the site names
        finger1_name, finger2_name = self.find_finger(site_name1, site_name2)

        # Assign q1 and q2 to the respective fingers
        if finger1_name == 'index':
            q_index = q1
        elif finger1_name == 'middle':
            q_middle = q1
        elif finger1_name == 'ring':
            q_ring = q1
        elif finger1_name == 'thumb':
            q_thumb = q1

        if finger2_name == 'index':
            q_index = q2
        elif finger2_name == 'middle':
            q_middle = q2
        elif finger2_name == 'ring':
            q_ring = q2
        elif finger2_name == 'thumb':
            q_thumb = q2
        
        # q=q_index+q_middle+q_ring+q_thumb
        q = np.hstack([q_index, q_middle, q_ring, q_thumb])
        # print(q)
        return np.array(q)
    



