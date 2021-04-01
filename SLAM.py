from matplotlib.pyplot import plot, show, scatter
from math import sin, cos, pi, sqrt
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


def get_joint_position(joint_handle):
    return_code, position = sim.simxGetJointPosition(clientID, joint_handle, sim.simx_opmode_blocking)
    if not return_code:
        return position


def read_proximity_sensor(sensor_handle):
    return_tuple = sim.simxReadProximitySensor(clientID, sensor_handle, sim.simx_opmode_blocking)
    return_code = return_tuple[0]
    if not return_code:
        detection_state = return_tuple[1]
        detected_distance = return_tuple[2][2]
        if not detection_state or detected_distance < 0:
            detected_distance = None
        return detected_distance


def motors_speed(added_speed):
    speed = 5
    if added_speed == 'right':
        speed = speed/2
        sim.simxSetJointTargetVelocity(clientID, motor_back_left, speed, sim.simx_opmode_streaming)
        sim.simxSetJointTargetVelocity(clientID, motor_back_right, - speed, sim.simx_opmode_streaming)
        sim.simxSetJointTargetVelocity(clientID, motor_front_left, speed, sim.simx_opmode_streaming)
        sim.simxSetJointTargetVelocity(clientID, motor_front_right, - speed, sim.simx_opmode_streaming)
    elif added_speed == 'left':
        speed = speed/2
        sim.simxSetJointTargetVelocity(clientID, motor_back_left, - speed, sim.simx_opmode_streaming)
        sim.simxSetJointTargetVelocity(clientID, motor_back_right, speed, sim.simx_opmode_streaming)
        sim.simxSetJointTargetVelocity(clientID, motor_front_left, - speed, sim.simx_opmode_streaming)
        sim.simxSetJointTargetVelocity(clientID, motor_front_right, speed, sim.simx_opmode_streaming)
    elif added_speed == 'stop':
        sim.simxSetJointTargetVelocity(clientID, motor_back_left, 0, sim.simx_opmode_streaming)
        sim.simxSetJointTargetVelocity(clientID, motor_back_right, 0, sim.simx_opmode_streaming)
        sim.simxSetJointTargetVelocity(clientID, motor_front_left, 0, sim.simx_opmode_streaming)
        sim.simxSetJointTargetVelocity(clientID, motor_front_right, 0, sim.simx_opmode_streaming)
    else:
        sim.simxSetJointTargetVelocity(clientID, motor_back_left, speed + added_speed, sim.simx_opmode_streaming)
        sim.simxSetJointTargetVelocity(clientID, motor_back_right, speed - added_speed, sim.simx_opmode_streaming)
        sim.simxSetJointTargetVelocity(clientID, motor_front_left, speed + added_speed, sim.simx_opmode_streaming)
        sim.simxSetJointTargetVelocity(clientID, motor_front_right, speed - added_speed, sim.simx_opmode_streaming)


def angle_step(angle_now, angle_prev):
    step = angle_now - angle_prev
    if step > pi:
        step = angle_now - 2*pi - angle_prev
    elif step < -pi:
        step = angle_now + 2*pi - angle_prev
    return step


print('Program started')
sim.simxFinish(-1)  # just in case, close all opened connections
clientID = sim.simxStart('127.0.0.1', 19999, True, True, 5000, 5)  # Connect to CoppeliaSim
if clientID != -1:
    print('Connected to remote API server')

    motor_back_left = get_object_handle('BL_joint')
    motor_back_right = get_object_handle('BR_joint')
    motor_front_left = get_object_handle('FL_joint')
    motor_front_right = get_object_handle('FR_joint')
    sensor_right = get_object_handle('FR_sensor')
    sensor_left = get_object_handle('FL_sensor')
    base = get_object_handle('Base')
    k_p = 8
    list_position_right = []
    list_position_left = []
    list_distance_right = []
    list_distance_left = []
    for i in range(500):
        position_right = get_joint_position(motor_front_right) + pi
        position_left = get_joint_position(motor_front_left) + pi
        distance_right = read_proximity_sensor(sensor_right)
        distance_left = read_proximity_sensor(sensor_left)
        list_position_right.append(position_right)
        list_position_left.append(position_left)
        list_distance_right.append(distance_right)
        list_distance_left.append(distance_left)
        if distance_right is None:
            motors_speed('right')
        elif distance_left is None:
            motors_speed('left')
        else:
            diff = distance_right - distance_left
            add_speed = k_p*diff
            motors_speed(add_speed)
    motors_speed('stop')
    for i in range(len(list_position_right)):
        print(list_position_right[i], list_position_left[i], list_distance_right[i], list_distance_left[i])

    # Before closing the connection to CoppeliaSim, make sure that the last command sent out had time to arrive.
    # You can guarantee this with (for example):
    sim.simxGetPingTime(clientID)

    # Now close the connection to CoppeliaSim:
    sim.simxFinish(clientID)
else:
    print('Failed connecting to remote API server')
print('Program ended')
