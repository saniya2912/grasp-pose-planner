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
        print(q)
        return np.array(q)
    
