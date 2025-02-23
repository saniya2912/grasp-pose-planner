import numpy as np
from main_grasptraj import *
import time
import mujoco
from leap_hand_utils.dynamixel_client import *
import numpy as np


#single object
leap_hand = LeapNode_Poscontrol()
pos=leap_hand.read_pos()
print('real_pos',pos)


# Load the model
model = mujoco.MjModel.from_xml_path('/home/saniya/Projects/grasp-pose-planner/model/leaphand.xml')
data = mujoco.MjData(model)

indexik=OnlyPosIK('/home/saniya/Projects/grasp-pose-planner/model/index_for_ik.xml')
middleik=OnlyPosIK('/home/saniya/Projects/grasp-pose-planner/model/middle_for_ik.xml')
ringik=OnlyPosIK('/home/saniya/Projects/grasp-pose-planner/model/ring_for_ik.xml')
thumbik=OnlyPosIK('/home/saniya/Projects/grasp-pose-planner/model/thumb_for_ik.xml')

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

q_mujoco=grasppose.joint_angles(site1_name,site2_name,q1,q2)
print('q_mujoco', q_mujoco)

q_real = q_mujoco
q_real[0], q_real[1] = q_real[1], q_real[0]

print('q_real', q_real)

while True:
    leap_hand.set_allegro(q_real)



# Launch the viewer
# with mujoco.viewer.launch_passive(model, data) as viewer:
#     while True:
#         data.qpos[0:16]=grasppose.joint_angles(site1_name,site2_name,q1,q2)
#         # Step the simulation
#         mujoco.mj_step(model, data)
#         print(data.qpos)
#         viewer.sync()



# def J(model,data,site_name):
# 	# model=mujoco.MjModel.from_xml_path(xml_path)
# 	# data = mujoco.MjData(model)
# 	mujoco.mj_forward(model, data)
# 	jacp = np.zeros((3, model.nv))  # translation jacobian
# 	jacr = np.zeros((3, model.nv))

# 	site_id=model.site(site_name).id
# 	mujoco.mj_jacSite(model, data, jacp, jacr, site_id)

# 	return np.vstack((jacp, jacr))


# pos_index= pos[0:4].reshape(4)
# pos_index_mujoco=[pos_index[1], pos_index[0], pos_index[2], pos_index[3]]

# pos_thumb=pos_thumb_mujoco= pos[12:16].reshape(4)


# index_model_path = '/home/saniya/Projects/grasp-pose-planner/model/index_for_ik.xml'

# index_m = mujoco.MjModel.from_xml_path(index_model_path)
# index_d = mujoco.MjData(index_m)


# index_d.qpos= pos_index_mujoco
# mujoco.mj_forward(index_m, index_d)
# index_J=J(index_m,index_d,'contact_index')


# thumb_model_path = '/home/saniya/LEAP/leap_hand_mujoco/model/leap hand/thumb.xml'

# thumb_m = mujoco.MjModel.from_xml_path(thumb_model_path)
# thumb_d = mujoco.MjData(thumb_m)

# thumb_d.qpos=pos_thumb_mujoco
# mujoco.mj_forward(thumb_m, thumb_d)
# thumb_J=J(thumb_m,thumb_d,'contact_thumb')



# pos_middle= pos2[4:8].reshape(4)
# pos_middle_mujoco=[pos_middle[1], pos_middle[0], pos_middle[2], pos_middle[3]]

# middle_model_path = '/home/saniya/LEAP/leap_hand_mujoco/model/leap hand/middle.xml'

# middle_m = mujoco.MjModel.from_xml_path(middle_model_path)
# middle_d = mujoco.MjData(middle_m)


# middle_d.qpos= pos_middle_mujoco
# mujoco.mj_forward(middle_m, middle_d)
# middle_J=J(middle_m,middle_d,'contact_middle')



# pos_tertiary= pos[8:12].reshape(4)
# pos_tertiary_mujoco=[pos_tertiary[1], pos_tertiary[0], pos_tertiary[2], pos_tertiary[3]]

# tertiary_model_path = '/home/saniya/LEAP/leap_hand_mujoco/model/leap hand/tertiary.xml'

# tertiary_m = mujoco.MjModel.from_xml_path(tertiary_model_path)
# tertiary_d = mujoco.MjData(tertiary_m)


# tertiary_d.qpos= pos_tertiary_mujoco
# mujoco.mj_forward(tertiary_m, tertiary_d)
# tertiary_J=J(tertiary_m,tertiary_d,'contact_tertiary')


# pos_index2= pos2[0:4].reshape(4)
# pos_index2_mujoco=[pos_index2[1], pos_index2[0], pos_index2[2], pos_index2[3]]

# pos_thumb2=pos_thumb2_mujoco= pos2[-4:].reshape(4)


# index_model_path = '/home/saniya/LEAP/leap_hand_mujoco/model/leap hand/index_finger.xml'

# index_m = mujoco.MjModel.from_xml_path(index_model_path)
# index_d = mujoco.MjData(index_m)


# index_d.qpos= pos_index2_mujoco
# mujoco.mj_forward(index_m, index_d)
# index_J2=J(index_m,index_d,'contact_index')


# thumb_model_path = '/home/saniya/LEAP/leap_hand_mujoco/model/leap hand/thumb.xml'

# thumb_m = mujoco.MjModel.from_xml_path(thumb_model_path)
# thumb_d = mujoco.MjData(thumb_m)

# thumb_d.qpos=pos_thumb2_mujoco
# mujoco.mj_forward(thumb_m, thumb_d)
# thumb_J2=J(thumb_m,thumb_d,'contact_thumb')

# # print(index_J2)
# # print(thumb_J2)
# F_index1 = np.reshape([-0.15, 0, 0, 0, 0, 0], [6, 1])
# F_thumb1=np.reshape([0.15, 0, 0, 0, 0, 0], [6, 1])

# # Compute torque values
# Tau_index1 = index_J.T @ F_index1
# Tau_index1[[0, 1]] = Tau_index1[[1, 0]]

# Tau_thumb1 = thumb_J.T @ F_thumb1

# # Convert torque values to float
# Tau_index1 = [float(torque[0]) for torque in Tau_index1]
# Tau_thumb1 = [float(torque[0]) for torque in Tau_thumb1]


# F_index2 = np.reshape([-0.1, 0, 0, 0, 0, 0], [6, 1])
# F_thumb2=np.reshape([0.1, 0, 0, 0, 0, 0], [6, 1])

# # Compute torque values
# Tau_index2 = index_J.T @ F_index2
# Tau_index2[[0, 1]] = Tau_index2[[1, 0]]

# Tau_thumb2 = thumb_J.T @ F_thumb2

# # Convert torque values to float
# Tau_index2 = [float(torque[0]) for torque in Tau_index2]
# Tau_thumb2 = [float(torque[0]) for torque in Tau_thumb2]


# leap_pos = LeapNode_Poscontrol()

# q0 = np.zeros(16) # 16-element array
# v0 = np.zeros(16)  # initial velocity (array of zeros)
# q1 = pos.reshape(16)  # final positions
# v1 = np.zeros(16)  # final velocity (array of zeros)
# t0 = 3 # initial time
# t1 = 7

# # q2=np.array([a1,b1,c1,d1, 0,0,0,0,0,0,0,0,e1,f1,g1,h1])
# # v2=np.zeros(16)

# start_time = time.time()

# while time.time() - start_time < t0:
#     leap_pos.set_allegro(np.zeros(16))
#     # Create a new row to append
#     new_row = pd.DataFrame({
#         'Time': [time.time()],
#         'start_time': [start_time],
#         'Torque': [leap_pos.read_cur()],
#         'Position': [leap_pos.read_pos()],
#         'Velocity': [leap_pos.read_vel()],
#         'Control_mode': ['pos'],
#         'Subtask': ['start']
#     })

#     # Append the new row using concat
#     if not new_row.empty:
#         df = pd.concat([df, new_row], ignore_index=True)
#     time.sleep(0.03)

# while time.time() - start_time < t1 and time.time() - start_time>t0:
#     current_time=time.time()
#     # Ensure the current time falls within [t0, t1]
    
#     qd = leap_pos.cubic_trajectory(q0, v0, q1, v1, t0, t1, current_time)
#     leap_pos.set_allegro(qd)
#     # Create a new row to append
#     new_row = pd.DataFrame({
#         'Time': [time.time()],
#         'start_time': [start_time],
#         'Torque': [leap_pos.read_cur()],
#         'Position': [leap_pos.read_pos()],
#         'Velocity': [leap_pos.read_vel()],
#         'Control_mode': ['pos'],
#         'Subtask': ['touch_object1']
#     })

#     # Append the new row using concat
#     if not new_row.empty:
#         df = pd.concat([df, new_row], ignore_index=True)
#     # time.sleep(0.03)

# leap_torque=LeapNode_Taucontrol()


# # Apply torque
# while time.time() - start_time < 15 and time.time() - start_time > t1:
#     curr_pos = leap_torque.read_pos_leap()
    
#     # After updating Tau_thumb, store current position for the next time step
#     Tau_thumb1[0] = 0.3*((pos[12] - curr_pos[12]))
#     leap_torque.set_desired_torque(Tau_index1+ [0,-0.1,0,0,0,-0.1,0,0]+Tau_thumb1)
#     # Create a new row to append
#     new_row = pd.DataFrame({
#         'Time': [time.time()],
#         'start_time': [start_time],
#         'Torque': [leap_torque.read_cur()],
#         'Position': [leap_torque.read_pos()],
#         'Velocity': [leap_torque.read_vel()],
#         'Control_mode': ['Tau'],
#         'Subtask': ['grasp_object1']
#     })

#     # Append the new row using concat
#     if not new_row.empty:
#         df = pd.concat([df, new_row], ignore_index=True)

# start_time=time.time()
# while time.time() - start_time < 2 and time.time() - start_time > 0:
    
#     leap_torque.set_desired_torque(Tau_index2+ [0,-0.1,0,0,0,-0.1,0,0]+Tau_thumb2)
#     new_row = pd.DataFrame({
#         'Time': [time.time()],
#         'start_time': [start_time],
#         'Torque': [leap_torque.read_cur()],
#         'Position': [leap_torque.read_pos()],
#         'Velocity': [leap_torque.read_vel()],
#         'Control_mode': ['Tau'],
#         'Subtask': ['readyfordrop']
#     })

#     # Append the new row using concat
#     if not new_row.empty:
#         df = pd.concat([df, new_row], ignore_index=True)

# # # Scaling factor for gradual reduction
# # scale_factor = 0.99  # Reduce by 1% in each loop iteration

# scale_factor = 0.999  # Reduce by 1% in each loop iteration
# F_index=F_index2
# F_thumb=F_thumb2

# start_time=time.time()

# while 0 <= (time.time() - start_time) <= 2:
#     # Gradually reduce the forces
#     F_index = scale_factor * F_index
#     F_thumb = scale_factor * F_thumb

#     # Recompute the torque values
#     Tau_index = index_J.T @ F_index
#     Tau_index[[0, 1]] = Tau_index[[1, 0]]

#     Tau_thumb = thumb_J.T @ F_thumb

#     # Convert torque values to float
#     Tau_index = [float(torque[0]) for torque in Tau_index]
#     Tau_thumb = [float(torque[0]) for torque in Tau_thumb]

#     # Apply the updated torque
#     leap_torque.set_desired_torque(Tau_index + [0, -0.1, 0, 0, 0, -0.1, 0, 0] + Tau_thumb)
#     new_row = pd.DataFrame({
#         'Time': [time.time()],
#         'start_time': [start_time],
#         'Torque': [leap_torque.read_cur()],
#         'Position': [leap_torque.read_pos()],
#         'Velocity': [leap_torque.read_vel()],
#         'Control_mode': ['Tau'],
#         'Subtask': ['dropping']
#     })

#     # Append the new row using concat
#     if not new_row.empty:
#         df = pd.concat([df, new_row], ignore_index=True)

# leap_pos=LeapNode_Poscontrol()

# while 2 <= (time.time() - start_time) <= 4:
    
#     leap_pos.set_allegro([0,np.pi/2,0,0,0,0,0,0,0,0,0,0,np.pi/2,0,0,0])
#     new_row = pd.DataFrame({
#         'Time': [time.time()],
#         'start_time': [start_time],
#         'Torque': [leap_pos.read_cur()],
#         'Position': [leap_pos.read_pos()],
#         'Velocity': [leap_pos.read_vel()],
#         'Control_mode': ['pos'],
#         'Subtask': ['pi/2']
#     })

#     # Append the new row using concat
#     if not new_row.empty:
#         df = pd.concat([df, new_row], ignore_index=True)

# leap_pos=LeapNode_Poscontrol()
# start_time = time.time()

# while time.time() - start_time < 5 and time.time() - start_time>0:
    
#     # Ensure the current time falls within [t0, t1]
    
#     leap_pos.set_allegro(pos2)
#     new_row = pd.DataFrame({
#         'Time': [time.time()],
#         'start_time': [start_time],
#         'Torque': [leap_pos.read_cur()],
#         'Position': [leap_pos.read_pos()],
#         'Velocity': [leap_pos.read_vel()],
#         'Control_mode': ['pos'],
#         'Subtask': ['touch_in_palm']
#     })

#     # Append the new row using concat
#     if not new_row.empty:
#         df = pd.concat([df, new_row], ignore_index=True)
    

# leap_torque=LeapNode_Taucontrol()
# start_time = time.time()



# F_middle = np.reshape([0, 0, 0.25, 0, 0, 0], [6, 1])
# F_tertiary = np.reshape([0, 0, -0.25, 0, 0, 0], [6, 1])

# # Compute torque values
# Tau_middle = middle_J.T @ F_middle
# Tau_middle[[0, 1]] = Tau_middle[[1, 0]]

# # Convert torque values to float
# Tau_middle = [float(torque[0]) for torque in Tau_middle]

# Tau_tertiary = tertiary_J.T @ F_tertiary
# Tau_tertiary[[0, 1]] = Tau_tertiary[[1, 0]]

# # Convert torque values to float
# Tau_tertiary = [float(torque[0]) for torque in Tau_tertiary]

    
# while time.time() - start_time < 10 and time.time()>0:
#     leap_torque.set_desired_torque([0,0,0,0] + Tau_middle + Tau_tertiary+ [0,0,0,0])
#     new_row = pd.DataFrame({
#         'Time': [time.time()],
#         'start_time': [start_time],
#         'Torque': [leap_torque.read_cur()],
#         'Position': [leap_torque.read_pos()],
#         'Velocity': [leap_torque.read_vel()],
#         'Control_mode': ['Tau'],
#         'Subtask': ['grasp_in_palm']
#     })

#     # Append the new row using concat
#     if not new_row.empty:
#         df = pd.concat([df, new_row], ignore_index=True)



    
# start_time = time.time()


# F_index_final = np.reshape([-0.15, 0, 0, 0, 0, 0], [6, 1])
# F_thumb_final = np.reshape([0.15, 0, 0, 0, 0, 0], [6, 1])

# # Compute torque values
# Tau_index_final = index_J2.T @ F_index_final
# Tau_index_final[[0, 1]] = Tau_index_final[[1, 0]]

# # Convert torque values to float
# Tau_index_final = [float(torque[0]) for torque in Tau_index_final]

# Tau_thumb_final = thumb_J2.T @ F_thumb_final

# # Convert torque values to float
# Tau_thumb_final = [float(torque[0]) for torque in Tau_thumb_final]

    
# while time.time() - start_time < 2 and time.time()>0:
#     leap_torque.set_desired_torque([0,0,0,0] + Tau_middle + Tau_tertiary+ [0,0,0,0])
#     new_row = pd.DataFrame({
#         'Time': [time.time()],
#         'start_time': [start_time],
#         'Torque': [leap_torque.read_cur()],
#         'Position': [leap_torque.read_pos()],
#         'Velocity': [leap_torque.read_vel()],
#         'Control_mode': ['Tau'],
#         'Subtask': ['second_object_grasp']
#     })

#     # Append the new row using concat
#     if not new_row.empty:
#         df = pd.concat([df, new_row], ignore_index=True)

# while time.time() - start_time < 10 and time.time()>2:
#     leap_torque.set_desired_torque(Tau_index_final + Tau_middle + Tau_tertiary+ Tau_thumb_final)
#     new_row = pd.DataFrame({
#         'Time': [time.time()],
#         'start_time': [start_time],
#         'Torque': [leap_torque.read_cur()],
#         'Position': [leap_torque.read_pos()],
#         'Velocity': [leap_torque.read_vel()],
#         'Control_mode': ['Tau'],
#         'Subtask': ['second_object_grasp']
#     })

#     # Append the new row using concat
#     if not new_row.empty:
#         df = pd.concat([df, new_row], ignore_index=True)