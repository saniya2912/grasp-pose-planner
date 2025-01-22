import mujoco
import mujoco.viewer
import numpy as np
from main import OnlyPosIK

# Load the model
model = mujoco.MjModel.from_xml_path('/home/iitgn-robotics/Saniya/grasp-pose-planner/model/leaphand.xml')
data = mujoco.MjData(model)

indexik=OnlyPosIK('/home/iitgn-robotics/Saniya/grasp-pose-planner/model/index_for_ik.xml')
thumbik=OnlyPosIK('/home/iitgn-robotics/Saniya/grasp-pose-planner/model/thumb_for_ik.xml')

# Set a target position (example)
contact_index=data.site(model.site('obj_contact_index').id).xpos.reshape(3) 
contact_thumb=data.site(model.site('obj_contact_thumb').id).xpos.reshape(3) 

# Calculate joint angles to reach the target
qindex = indexik.calculate(contact_index, '1_4')
qthumb= thumbik.calculate(contact_thumb, '4_3')

# Launch the viewer
with mujoco.viewer.launch_passive(model, data) as viewer:
    while True:
        data.qpos[0:4]=qindex
        data.qpos[4:8]=[0,0,0,0]
        data.qpos[8:12]=[0,0,0,0]
        data.qpos[12:16]=qthumb
        # Step the simulation
        mujoco.mj_step(model, data)
        print(data.qpos)
        viewer.sync()
