import numpy as np
import time
import sim
import PRM


def move_start_goal(clientID, target_handle, quad_handle, start, goal):
    obs = np.array([[0, 0, 5, 1.5, 1.5, 1.5]])
    # path = PRM.generate_path(start, goal, 10, obs)
    path = [(0, -4, 5), (0.8105243726757525, 1.0331600503174894, 3.8575081632430646), (1.0406774097953697, 3.2213277352831113, 3.1361695850086555), (0, 4, 5)]
    for i in range(len(path)):
        move_target(clientID, target_handle, path[i])
        cur_pos = get_object_position(clientID, quad_handle)
        p_cur = np.array(cur_pos)
        p_goal = np.array(path[i])
        while (p_cur < p_goal - 0.25).any() or (p_cur > p_goal + 0.25).any():
            time.sleep(1)
            cur_pos = get_object_position(clientID, quad_handle)
            p_cur = np.array(cur_pos)



def move_target(clientID, target_handle, target_position):
    # Set the position of the target object
    sim.simxSetObjectPosition(clientID, target_handle, -1, target_position, sim.simx_opmode_oneshot)



def get_object_position(clientID, object_handle):
    # Get the position of the object
    _, object_position = sim.simxGetObjectPosition(clientID, object_handle, -1, sim.simx_opmode_blocking)
    
    return object_position


def main():
    # Connect to CoppeliaSim
    sim.simxFinish(-1)
    clientID = sim.simxStart('127.0.0.1', 19997, True, True, 5000, 5)
    
    if clientID == -1:
        print('Failed to connect to CoppeliaSim')
        return
    
    print('Connected to CoppeliaSim')
    
    # Get handle of the helicopter object
    _, target_handle = sim.simxGetObjectHandle(clientID, 'Quadcopter_target', sim.simx_opmode_oneshot_wait)
    _, quad_handle = sim.simxGetObjectHandle(clientID, 'Quadcopter', sim.simx_opmode_oneshot_wait)

    if target_handle == -1 or quad_handle == -1:
        print('Failed to get handle of the target object')
        sim.simxFinish(clientID)
        return
    
    start = (0, -4, 5)
    goal = (0, 4, 5)
    move_start_goal(clientID, target_handle, quad_handle, start, goal)

    # Disconnect from CoppeliaSim
    sim.simxFinish(clientID)
    print('Disconnected from CoppeliaSim')

if __name__ == '__main__':
    main()
