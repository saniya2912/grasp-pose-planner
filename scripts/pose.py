# import mujoco
# import numpy as np
# from scipy.optimize import minimize

# # Load the MuJoCo model
# model = mujoco.MjModel.from_xml_path('/home/iitgn-robotics/Saniya/grasp-pose-planner/model/leaphand.xml')
# data = mujoco.MjData(model)

# # Joint and zone definitions
# finger_zones = {
#     'fingertip': 4,
#     'dip': 3,
#     'pip': 2,
#     'mcp': 1
# }
# palm_zone = 0

# # Joint limit constraints
# def joint_limits(joint_idx):
#     return (model.jnt_range[joint_idx, 0], model.jnt_range[joint_idx, 1])

# # Objective function: Minimize joint movement and achieve target pose
# def objective(qpos, initial_qpos, target_pose):
#     # Update model with current qpos
#     data.qpos[:] = qpos
#     mujoco.mj_fwdPosition(model, data)
    
#     # Compute the difference between current and target poses
#     current_pose = data.body('fingertip').xpos
#     pose_error = np.sum((current_pose - target_pose)**2)
    
#     # Joint movement penalty
#     movement_penalty = np.sum((qpos - initial_qpos)**2)
    
#     return pose_error + movement_penalty

# # Define the constraint function for joint limits
# def joint_limit_constraint(qpos):
#     return [
#         (qpos[j] - model.jnt_range[j, 0], model.jnt_range[j, 1] - qpos[j])
#         for j in range(model.njnt)
#     ]

# # Inverse Kinematics function
# def inverse_kinematics(target_pose):
#     def ik_objective(qpos):
#         data.qpos[:] = qpos
#         mujoco.mj_fwdPosition(model, data)
#         current_pose = data.body('fingertip').xpos
#         return np.sum((current_pose - target_pose)**2)
    
#     qpos_initial = np.zeros(model.njnt)
#     result = minimize(
#         fun=ik_objective,
#         x0=qpos_initial,
#         bounds=[joint_limits(j) for j in range(model.njnt)],
#         method='SLSQP',
#         options={'disp': True}
#     )
    
#     if result.success:
#         return result.x
#     else:
#         print("IK Optimization failed.")
#         return None

# # Planner function
# def grasp_pose_planner(target_zones, object_pose, initial_qpos):
#     # Initial guess for the optimizer
#     qpos_initial = np.copy(initial_qpos)
    
#     # Constraints for optimization
#     constraints = [
#         {'type': 'ineq', 'fun': lambda qpos: joint_limit_constraint(qpos)}
#     ]
    
#     # Perform optimization
#     result = minimize(
#         fun=objective,
#         x0=qpos_initial,
#         args=(initial_qpos, object_pose),
#         constraints=constraints,
#         method='SLSQP',
#         options={'disp': True}
#     )
    
#     if result.success:
#         print("Optimal joint configuration found:", result.x)
#         return result.x
#     else:
#         print("Optimization failed.")
#         return None

# # Example usage
# initial_qpos = np.zeros(model.njnt)
# # Assuming `target_zones` and `object_pose` are provided
# # target_zones = ...
# # object_pose = ...
# # optimal_qpos = grasp_pose_planner(target_zones, object_pose, initial_qpos)


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
# Joint and zone definitions
finger_zones = {
    'fingertip': 4,
    'dip': 3,
    'pip': 2,
    'mcp': 1
}
palm_zone = 0

# Joint limit constraints
def joint_limits(joint_idx, model):
    return (model.jnt_range[joint_idx, 0], model.jnt_range[joint_idx, 1])

# Planner function
def grasp_pose_planner(target_zones, object_pose, initial_qpos, ik_solver):
    # Initial guess for the optimizer
    qpos_initial = np.copy(initial_qpos)
    
    # Use IK to get close to the desired pose
    site_name = 'fingertip'  # Assuming we're focusing on the fingertip for this example
    goal_pos = object_pose[:3]  # Extracting position from the object pose
    goal_rot = object_pose[3:].reshape(3, 3)  # Extracting rotation matrix
    ik_result = ik_solver.calculate(goal_pos, goal_rot, site_name)
    
    if ik_result is not None:
        print("IK-based initial joint configuration:", ik_result)
        return ik_result
    else:
        print("IK failed to find a solution.")
        return None

# Example usage
xml_path = 'leaphand.xml'
ik_solver = OnlyPosIK(xml_path)
initial_qpos = np.zeros(ik_solver.model.njnt)
# Assuming `target_zones` and `object_pose` are provided
# target_zones = ...
# object_pose = ...
# optimal_qpos = grasp_pose_planner(target_zones, object_pose, initial_qpos, ik_solver)
