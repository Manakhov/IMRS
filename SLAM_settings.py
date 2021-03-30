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


def get_object_orientation(object_name):
    return_code, euler_angles = sim.simxGetObjectOrientation(clientID, object_name, -1, sim.simx_opmode_blocking)
    if not return_code:
        return euler_angles[2]


def get_object_position(object_handle):
    return_code, position = sim.simxGetObjectPosition(clientID, object_handle, -1, sim.simx_opmode_blocking)
    if not return_code:
        return position[:2]


def read_proximity_sensor(sensor_handle):
    return_tuple = sim.simxReadProximitySensor(clientID, sensor_handle, sim.simx_opmode_blocking)
    return_code = return_tuple[0]
    if not return_code:
        detection_state = return_tuple[1]
        detected_point = return_tuple[2]
        return detection_state, detected_point[2]


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
    distance = 1
    way = 0
    angle = 0
    rotation = 5*pi/2
    base_rotation = 0
    diff_angle = 0
    motors_speed(0)
    x_start, y_start = get_object_position(base)
    y_prev = y_start
    position_right_prev = get_joint_position(motor_front_right) + pi
    while way < distance:
        x_now, y_now = get_object_position(base)
        position_right = get_joint_position(motor_front_right) + pi
        y_step = abs(y_now - y_prev)
        right_step = angle_step(position_right, position_right_prev)
        way = way + y_step
        angle = angle + right_step
        y_prev = y_now
        position_right_prev = position_right
    motors_speed('stop')
    x_finish, y_finish = get_object_position(base)
    real_way = abs(y_finish - y_start)
    position_right = get_joint_position(motor_front_right) + pi
    right_step = angle_step(position_right, position_right_prev)
    angle = angle + right_step
    k_pos = real_way/angle  # position quotient (m/rad)
    print('k_pos = ', k_pos)
    motors_speed('left')
    base_orientation_start = get_object_orientation(base) + pi
    base_orientation_prev = base_orientation_start
    position_right_prev = get_joint_position(motor_front_right) + pi
    position_left_prev = get_joint_position(motor_front_left) + pi
    while base_rotation < rotation:
        base_orientation = get_object_orientation(base) + pi
        position_right = get_joint_position(motor_front_right) + pi
        position_left = get_joint_position(motor_front_left) + pi
        base_step = angle_step(base_orientation, base_orientation_prev)
        right_step = angle_step(position_right, position_right_prev)
        left_step = angle_step(position_left, position_left_prev)
        diff_step = right_step - left_step
        base_rotation = base_rotation + base_step
        diff_angle = diff_angle + diff_step
        base_orientation_prev = base_orientation
        position_right_prev = position_right
        position_left_prev = position_left
    motors_speed('stop')
    base_orientation_finish = get_object_orientation(base) + pi
    real_rotation = base_orientation_finish + 2 * pi - base_orientation_start
    position_right = get_joint_position(motor_front_right) + pi
    position_left = get_joint_position(motor_front_left) + pi
    right_step = angle_step(position_right, position_right_prev)
    left_step = angle_step(position_left, position_left_prev)
    diff_step = right_step - left_step
    diff_angle = diff_angle + diff_step
    k_ori = real_rotation/diff_angle  # orientation quotient (rad/rad)
    print('k_ori = ', k_ori)

    # Before closing the connection to CoppeliaSim, make sure that the last command sent out had time to arrive.
    # You can guarantee this with (for example):
    sim.simxGetPingTime(clientID)

    # Now close the connection to CoppeliaSim:
    sim.simxFinish(clientID)
else:
    print('Failed connecting to remote API server')
print('Program ended')
