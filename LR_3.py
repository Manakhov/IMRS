from time import sleep
from math import pi
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
        detection_state = return_tuple[1]
        detected_point = return_tuple[2]
        return detection_state, detected_point[2]


def read_vision_sensor(sensor_handle):
    return_tuple = sim.simxReadVisionSensor(clientID, sensor_handle, sim.simx_opmode_blocking)
    return_code = return_tuple[0]
    if not return_code:
        intensity = return_tuple[2]
        return intensity[0][2]


def set_joint_target_position(config, joint_list):
    for i in range(len(config)):
        sim.simxSetJointTargetPosition(clientID, joint_list[i], config[i], sim.simx_opmode_oneshot)


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
    joint_list = [base_axis, axis1_2, axis2_3, axis3_4, axis4_5]
    proximity_sensor = get_object_handle('Proximity_sensor')
    vision_sensor = get_object_handle('Vision_sensor')
    start_config = [0, -pi/4, pi/4 + 0.2, 0, 0]
    set_joint_target_position(start_config, joint_list)
    base_angle = start_config[0]
    axis1_2_angle = start_config[1]
    axis2_3_angle = start_config[2]
    finish_angle = start_config[4]
    result = False
    while base_angle < 2*pi:
        if finish_angle > 0:
            while finish_angle > 0:
                sim.simxSetJointTargetPosition(clientID, axis4_5, finish_angle, sim.simx_opmode_oneshot)
                sleep(0.001)
                if read_vision_sensor(vision_sensor) > 0.90:
                    result = True
                    break
                finish_angle = finish_angle - pi/300
        else:
            while finish_angle < pi/3:
                sim.simxSetJointTargetPosition(clientID, axis4_5, finish_angle, sim.simx_opmode_oneshot)
                sleep(0.001)
                if read_vision_sensor(vision_sensor) > 0.90:
                    result = True
                    break
                finish_angle = finish_angle + pi/300
        if result:
            break
        sim.simxSetJointTargetPosition(clientID, base_axis, base_angle, sim.simx_opmode_oneshot)
        sleep(0.001)
        base_angle = base_angle + pi/100
    while True:
        if read_vision_sensor(vision_sensor) > 0.90:
            axis1_2_angle = axis1_2_angle - 0.01
            sim.simxSetJointTargetPosition(clientID, axis1_2, axis1_2_angle, sim.simx_opmode_oneshot)
        else:
            axis2_3_angle = axis2_3_angle + 0.01
            finish_angle = finish_angle - 0.01
            sim.simxSetJointTargetPosition(clientID, axis2_3, axis2_3_angle, sim.simx_opmode_oneshot)
            sim.simxSetJointTargetPosition(clientID, axis4_5, finish_angle, sim.simx_opmode_oneshot)
        print("visio:", sim.simxReadVisionSensor(clientID, vision_sensor, sim.simx_opmode_blocking))
        # print("proxi:", sim.simxReadProximitySensor(clientID, proximity_sensor, sim.simx_opmode_blocking))
        sleep(0.001)

    # Before closing the connection to CoppeliaSim, make sure that the last command sent out had time to arrive.
    # You can guarantee this with (for example):
    sim.simxGetPingTime(clientID)

    # Now close the connection to CoppeliaSim:
    sim.simxFinish(clientID)
else:
    print('Failed connecting to remote API server')
print('Program ended')
