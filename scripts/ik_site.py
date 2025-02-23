import mujoco
import mujoco.viewer
import numpy as np
from main import OnlyPosIK,GraspPose

# Load the model
model = mujoco.MjModel.from_xml_path('/home/iitgn-robotics/Saniya/grasp-pose-planner/model/leaphand.xml')
data = mujoco.MjData(model)

indexik=OnlyPosIK('/home/iitgn-robotics/Saniya/grasp-pose-planner/model/index_for_ik.xml')
middleik=OnlyPosIK('/home/iitgn-robotics/Saniya/grasp-pose-planner/model/middle_for_ik.xml')
ringik=OnlyPosIK('/home/iitgn-robotics/Saniya/grasp-pose-planner/model/ring_for_ik.xml')
thumbik=OnlyPosIK('/home/iitgn-robotics/Saniya/grasp-pose-planner/model/thumb_for_ik.xml')

grasppose=GraspPose()
site1_name='1_4'
site2_name='4_3'

mujoco.mj_step(model, data)

# Set a target position (example)
contact_1=data.site(model.site('obj_contact_1').id).xpos.reshape(3) 
contact_2=data.site(model.site('obj_contact_2').id).xpos.reshape(3) 

finger1str,finger2str=grasppose.ik_name(site1_name,site2_name)

finger1ik = globals().get(finger1str, None)
finger2ik = globals().get(finger2str, None)

# Calculate joint angles to reach the target
q1 = finger1ik.calculate(contact_1, site1_name)
q2= finger2ik.calculate(contact_2, site2_name)

# Launch the viewer
with mujoco.viewer.launch_passive(model, data) as viewer:
    while True:
        data.qpos[0:16]=grasppose.joint_angles(site1_name,site2_name,q1,q2)
        # Step the simulation
        mujoco.mj_step(model, data)
        print(data.qpos)
        viewer.sync()
