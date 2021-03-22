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
    else:
        sim.simxSetJointTargetVelocity(clientID, motor_back_left, speed + added_speed, sim.simx_opmode_streaming)
        sim.simxSetJointTargetVelocity(clientID, motor_back_right, speed - added_speed, sim.simx_opmode_streaming)
        sim.simxSetJointTargetVelocity(clientID, motor_front_left, speed + added_speed, sim.simx_opmode_streaming)
        sim.simxSetJointTargetVelocity(clientID, motor_front_right, speed - added_speed, sim.simx_opmode_streaming)


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
    k_p = 9
    position_array = []
    position_right_array = []
    position_left_array = []
    x_array = []
    y_array = []
    x_right_array = []
    y_right_array = []
    x_left_array = []
    y_left_array = []
    dead_zone = 0.01/(cos(pi/4))
    prev_x_right = 0
    prev_y_right = 0
    prev_x_left = 0
    prev_y_left = 0
    prev_vector_right = 0
    prev_vector_left = 0
    for i in range(500):
        x, y = get_object_position(base)
        gamma = get_object_orientation(base) + pi
        state_right, distance_right = read_proximity_sensor(sensor_right)
        state_left, distance_left = read_proximity_sensor(sensor_left)
        position_array.append([x, y])
        if not state_right:
            motors_speed('right')
            while not state_right:
                state_right, distance_right = read_proximity_sensor(sensor_right)
        elif not state_left:
            motors_speed('left')
            while not state_left:
                state_left, distance_left = read_proximity_sensor(sensor_left)
        else:
            x_right = x + cos(gamma + pi/4)*(distance_right + dead_zone)
            y_right = y + sin(gamma + pi/4)*(distance_right + dead_zone)
            x_left = x - cos(gamma - pi/4)*(distance_left + dead_zone)
            y_left = y - sin(gamma - pi/4)*(distance_left + dead_zone)
            vector_right = sqrt((x_right - prev_x_right)**2 + (y_right - prev_y_right)**2)
            vector_left = sqrt((x_left - prev_x_left)**2 + (y_left - prev_y_left)**2)
            vector_right_diff = abs(vector_right - prev_vector_right)
            vector_left_diff = abs(vector_left - prev_vector_left)
            if vector_right_diff < 0.00001 and vector_left_diff < 0.00001:
                motors_speed('left')
                while distance_left < 0.3:
                    state_left, distance_left = read_proximity_sensor(sensor_left)
                continue
            if vector_right < 0.1:
                position_right_array.append([x_right, y_right])
            if vector_left < 0.1:
                position_left_array.append([x_left, y_left])
            diff = distance_right - distance_left
            add_speed = k_p*diff
            motors_speed(add_speed)
            prev_x_right = x_right
            prev_y_right = y_right
            prev_x_left = x_left
            prev_y_left = y_left
            prev_vector_right = vector_right
            prev_vector_left = vector_left
    for pos in position_array:
        x_array.append(pos[0])
        y_array.append(pos[1])
    for pos in position_right_array:
        x_right_array.append(pos[0])
        y_right_array.append(pos[1])
    for pos in position_left_array:
        x_left_array.append(pos[0])
        y_left_array.append(pos[1])
    plot(x_array, y_array)
    scatter(x_right_array, y_right_array)
    scatter(x_left_array, y_left_array)
    show()

    # Before closing the connection to CoppeliaSim, make sure that the last command sent out had time to arrive.
    # You can guarantee this with (for example):
    sim.simxGetPingTime(clientID)

    # Now close the connection to CoppeliaSim:
    sim.simxFinish(clientID)
else:
    print('Failed connecting to remote API server')
print('Program ended')
