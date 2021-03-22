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
    k_p = 8
    for i in range(1000):
        state_right, distance_right = read_proximity_sensor(sensor_right)
        state_left, distance_left = read_proximity_sensor(sensor_left)
        if not state_right:
            motors_speed('right')
            while not state_right:
                state_right, distance_right = read_proximity_sensor(sensor_right)
        elif not state_left:
            motors_speed('left')
            while not state_left:
                state_left, distance_left = read_proximity_sensor(sensor_left)
        else:
            diff = distance_right - distance_left
            add_speed = k_p*diff
            motors_speed(add_speed)

    # Before closing the connection to CoppeliaSim, make sure that the last command sent out had time to arrive.
    # You can guarantee this with (for example):
    sim.simxGetPingTime(clientID)

    # Now close the connection to CoppeliaSim:
    sim.simxFinish(clientID)
else:
    print('Failed connecting to remote API server')
print('Program ended')
