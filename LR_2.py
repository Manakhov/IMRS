from time import sleep
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


print('Program started')
sim.simxFinish(-1)  # just in case, close all opened connections
clientID = sim.simxStart('127.0.0.1', 19999, True, True, 5000, 5)  # Connect to CoppeliaSim
if clientID != -1:
    print('Connected to remote API server')

    model_base = get_object_handle('Gripper_Base')
    sensor = get_object_handle('Proximity_sensor')
    left_handle = get_object_handle('Left_joint')
    right_handle = get_object_handle('Right_joint')
    Detection = 0.04
    Dead_zone = 0.005
    Stop_time = 5
    result, start_distance = read_proximity_sensor(sensor)
    if result:
        start_distance = start_distance - Dead_zone
        if 0 < start_distance < Detection:
            sim.simxSetJointTargetVelocity(clientID, left_handle, 0.005, sim.simx_opmode_oneshot)
            sim.simxSetJointTargetVelocity(clientID, right_handle, 0.005, sim.simx_opmode_oneshot)
            while result:
                result, distance = read_proximity_sensor(sensor)
                if distance < Dead_zone:
                    break
            sim.simxSetJointTargetVelocity(clientID, left_handle, 0, sim.simx_opmode_oneshot)
            sim.simxSetJointTargetVelocity(clientID, right_handle, 0, sim.simx_opmode_oneshot)
            sleep(Stop_time)
            sim.simxSetJointTargetVelocity(clientID, left_handle, -0.005, sim.simx_opmode_oneshot)
            sim.simxSetJointTargetVelocity(clientID, right_handle, -0.005, sim.simx_opmode_oneshot)
            while True:
                result, distance = read_proximity_sensor(sensor)
                if result:
                    distance = distance - Dead_zone
                    if distance >= start_distance:
                        break
            sim.simxSetJointTargetVelocity(clientID, left_handle, 0, sim.simx_opmode_oneshot)
            sim.simxSetJointTargetVelocity(clientID, right_handle, 0, sim.simx_opmode_oneshot)

    # Before closing the connection to CoppeliaSim, make sure that the last command sent out had time to arrive.
    # You can guarantee this with (for example):
    sim.simxGetPingTime(clientID)

    # Now close the connection to CoppeliaSim:
    sim.simxFinish(clientID)
else:
    print('Failed connecting to remote API server')
print('Program ended')
