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
    
class GraspPose:
    def joint_limits(joint_idx, model):
        return (model.jnt_range[joint_idx, 0], model.jnt_range[joint_idx, 1])

    def contact_point(object_pose):
        return
