from time import sleep
from math import pi, sqrt, acos
try:
    import sim
except:
    print('--------------------------------------------------------------')
    print('"sim.py" could not be imported. This means very probably that')
    print('either "sim.py" or the remoteApi library could not be found.')
    print('Make sure both are in the same folder as this file,')
    print('or appropriately adjust the file "sim.py"')
    print('--------------------------------------------------------------')
    print('')


def get_object_handle(object_name):
    return_code, handle = sim.simxGetObjectHandle(clientID, object_name, sim.simx_opmode_blocking)
    if not return_code:
        return handle


def read_proximity_sensor(sensor_handle):
    return_tuple = sim.simxReadProximitySensor(clientID, sensor_handle, sim.simx_opmode_blocking)
    return_code = return_tuple[0]
    if not return_code:
        detected_point = return_tuple[2]
        return detected_point[2]


def read_vision_sensor(sensor_handle):
    return_tuple = sim.simxReadVisionSensor(clientID, sensor_handle, sim.simx_opmode_blocking)
    return_code = return_tuple[0]
    if not return_code:
        intensity = return_tuple[2]
        return intensity[0][2]


def set_joint_target_position(config, joint_list):
    for i in range(len(config)):
        sim.simxSetJointTargetPosition(clientID, joint_list[i], config[i], sim.simx_opmode_oneshot)
        sleep(0.001)


print('Program started')
sim.simxFinish(-1)  # just in case, close all opened connections
clientID = sim.simxStart('127.0.0.1', 19999, True, True, 5000, 5)  # Connect to CoppeliaSim
if clientID != -1:
    print('Connected to remote API server')

    base_axis = get_object_handle('Base_axis')
    axis1_2 = get_object_handle('Axis1_2')
    axis2_3 = get_object_handle('Axis2_3')
    axis3_4 = get_object_handle('Axis3_4')
    axis4_5 = get_object_handle('Axis4_5')
    proximity_sensor = get_object_handle('Proximity_sensor')
    vision_sensor = get_object_handle('Vision_sensor')
    joint_list = [base_axis, axis1_2, axis2_3, axis3_4, axis4_5]
    member_1 = 0.220/1.216
    member_2 = 0.532/1.216
    member_4 = 0.543/1.216
    member_5 = 0.092/1.216
    start_config = [0, -pi/7, pi/5, 0, pi/8]
    set_joint_target_position(start_config, joint_list)
    base_angle = start_config[0]
    axis1_2_angle = start_config[1]
    axis2_3_angle = start_config[2]
    axis3_4_angle = start_config[3]
    finish_angle = start_config[4]
    finish_low_range = finish_angle - pi/6
    finish_high_range = finish_angle + pi/8
    finish_step = (finish_high_range - finish_low_range)/100
    base_range = 2*pi
    base_step = base_range/100
    result = False
    finish_angle = finish_high_range
    while base_angle < base_range:
        if finish_angle <= finish_low_range or finish_angle >= finish_high_range:
            finish_step = - finish_step
            finish_angle = finish_angle + finish_step
            sim.simxSetJointTargetPosition(clientID, axis4_5, finish_angle, sim.simx_opmode_oneshot)
            sleep(0.001)
        while finish_low_range < finish_angle < finish_high_range:
            if read_vision_sensor(vision_sensor) > 0.9:
                result = True
                break
            finish_angle = finish_angle + finish_step
            sim.simxSetJointTargetPosition(clientID, axis4_5, finish_angle, sim.simx_opmode_oneshot)
            sleep(0.001)
        if result:
            break
        base_angle = base_angle + base_step
        sim.simxSetJointTargetPosition(clientID, base_axis, base_angle, sim.simx_opmode_oneshot)
        sleep(0.001)
    distance = read_proximity_sensor(proximity_sensor)
    a_1 = sqrt((distance - member_5)**2 - (member_2 + member_1)**2)
    if finish_angle > start_config[4]:
        a_1 = - a_1
    a = a_1 + member_4
    gip = sqrt(a**2 + (member_1 - member_5)**2)
    alpha = acos((member_1 - member_5)/gip)
    betta = acos((member_2**2 + gip**2 - member_4**2)/(2*member_2*gip))
    axis1_2_rot = pi - alpha - betta
    axis1_2_angle = axis1_2_angle - axis1_2_rot
    gamma = acos((member_2**2 + member_4**2 - gip**2)/(2*member_2*member_4))
    axis2_3_rot = pi/2 - gamma
    axis2_3_angle = axis2_3_angle - axis2_3_rot
    if finish_angle > start_config[4]:
        finish_angle_rot = 2*pi - alpha - betta - gamma - pi/2
    else:
        finish_angle_rot = 3*pi - alpha - betta - gamma - 3*pi/2
    finish_angle = start_config[4] - finish_angle_rot
    finish_config = [base_angle, axis1_2_angle, axis2_3_angle, axis3_4_angle, finish_angle]
    set_joint_target_position(finish_config, joint_list)

    # Before closing the connection to CoppeliaSim, make sure that the last command sent out had time to arrive.
    # You can guarantee this with (for example):
    sim.simxGetPingTime(clientID)

    # Now close the connection to CoppeliaSim:
    sim.simxFinish(clientID)
else:
    print('Failed connecting to remote API server')
print('Program ended')
