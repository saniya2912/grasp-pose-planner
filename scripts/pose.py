import mujoco
import numpy as np
from main import OnlyPosIK

def grasp_pose_planner(object_pose, hand_zone):
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
xml_path = '/home/iitgn-robotics/Saniya/grasp-pose-planner/model/leaphand.xml'
ik_solver = OnlyPosIK(xml_path)
initial_qpos = np.zeros(ik_solver.model.njnt)
