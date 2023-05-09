import numpy as np
import time
import sim
import PRM
import threading


"""def move_start_goal(clientID, quad_handle, start, goal):
    obs = np.array([[0, 0, 5, 1.5, 1.5, 1.5]])
    path = PRM.generate_path(start, goal, 10, obs)
    for i in range(len(path)):
        move_quad(clientID, quad_handle, path[i])
        cur_pos = get_object_position(clientID, quad_handle)
        p_cur = np.array(cur_pos)
        p_goal = np.array(path[i])
        while np.linalg.norm(p_goal - p_cur) > 0.25:
            print(f'Current position: {p_cur}, Goal position: {p_goal}, Distance: {np.linalg.norm(p_goal - p_cur)}')
            time.sleep(3)
            cur_pos = get_object_position(clientID, quad_handle)
            p_cur = np.array(cur_pos)

def move_quad(clientID, quad_handle, quad_position):
    # Set the position of the quadcopter directly
    sim.simxSetObjectPosition(clientID, quad_handle, -1, quad_position, sim.simx_opmode_blocking)"""


def move_start_goal(clientID, target_handle, quad_handle, start, goal):
    obs = np.array([[0, 0, 5, 1.5, 1.5, 1.5]])
    
    print('Generating path')
    path = PRM.generate_path(start, goal, 10, obs)
    print('Path generated:')

    print(path)
    for i in range(len(path)):
        move_target(clientID, target_handle, path[i])
        cur_pos = get_object_position(clientID, quad_handle)
        p_cur = np.array(cur_pos)
        p_goal = np.array(path[i])
        while np.linalg.norm(p_goal - p_cur) > 0.25:
            print(f'Current position: {p_cur}, Goal position: {p_goal}, Distance: {np.linalg.norm(p_goal - p_cur)}')
            time.sleep(3)
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

    # Get handle of the helicopter objects and their targets
    _, target1_handle = sim.simxGetObjectHandle(clientID, 'Quadcopter1_target', sim.simx_opmode_oneshot_wait)
    _, quad1_handle = sim.simxGetObjectHandle(clientID, 'Quadcopter1', sim.simx_opmode_oneshot_wait)

    _, target2_handle = sim.simxGetObjectHandle(clientID, 'Quadcopter2_target', sim.simx_opmode_oneshot_wait)
    _, quad2_handle = sim.simxGetObjectHandle(clientID, 'Quadcopter2', sim.simx_opmode_oneshot_wait)

    if target1_handle == -1 or quad1_handle == -1 or target2_handle == -1 or quad2_handle == -1:
        print('Failed to get handle of the target object')
        sim.simxFinish(clientID)
        return

    print('Successfully got handles')

    start = (0, -4, 5)
    goal = (0, 4, 5)

    # Create threads for each robot
    thread1 = threading.Thread(target=move_start_goal, args=(clientID, target1_handle, quad1_handle, start, goal))
    thread2 = threading.Thread(target=move_start_goal, args=(clientID, target2_handle, quad2_handle, goal, start))

    """thread1 = threading.Thread(target=move_start_goal, args=(clientID, quad1_handle, start, goal))
    thread2 = threading.Thread(target=move_start_goal, args=(clientID, quad2_handle, goal, start))"""


    # Start the threads
    thread1.start()
    thread2.start()

    # Wait for the threads to finish
    thread1.join()
    thread2.join()

    print('movement complete')

    # Disconnect from CoppeliaSim
    sim.simxFinish(clientID)
    print('Disconnected from CoppeliaSim')


if __name__ == '__main__':
    main()