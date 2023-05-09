import numpy as np
import time
import sim
import PRM
import threading


def move_start_goal(clientID, target_handle, quad_handle, quad2_handle, start, goal):
    obs = np.array([[0, 0, 5, 1.75, 1.75, 1.75], [0, 0, 7.5, 1.75, 1.75, 1.75], [0, 0, 2.5, 1.75, 1.75, 1.75], [-2.5, 0, 5, 1.75, 1.75, 1.75], [2.5, 0, 5, 1.75, 1.75, 1.75]])
    
    print('Generating path')
    # path = PRM.generate_path(start, goal, 15, obs)

    # for fast testing
    if goal == (0, -4, 5):
        path = [(0.9085846508150217, 2.942486993242058, 4.104828061215315), (3.489937193912617, 1.5305395863232727, 3.805358590750762), (0, -4, 5)]
    else:
        time.sleep(30)
        path = [(1.464013135579675, 1.5204426671146765, 3.9915166353731273), (4.128863747456995, 1.4157438358294065, 4.042325130220565), (1.464013135579675, 1.5204426671146765, 3.9915166353731273), (0, 4, 5)]
    print('Path generated:')

    print(path)
    for i in range(len(path)):
        move_target(clientID, target_handle, path[i])
        cur_pos = get_object_position(clientID, quad_handle)
        p_cur = np.array(cur_pos)
        p_goal = np.array(path[i])
        while np.linalg.norm(p_goal - p_cur) > 0.25:
            # print(f'Current position: {p_cur}, Goal position: {p_goal}, Distance: {np.linalg.norm(p_goal - p_cur)}')
            check_path(clientID, target_handle, quad_handle, quad2_handle, p_goal, p_cur)
            cur_pos = get_object_position(clientID, quad_handle)
            p_cur = np.array(cur_pos)

def move_target(clientID, target_handle, target_position):
    # Set the position of the target object
    sim.simxSetObjectPosition(clientID, target_handle, -1, target_position, sim.simx_opmode_oneshot)

def get_object_position(clientID, object_handle):
    # Get the position of the object
    _, object_position = sim.simxGetObjectPosition(clientID, object_handle, -1, sim.simx_opmode_blocking)
    
    return object_position

def check_path(clientID, target_handle, quad1_handle, quad2_handle, p_goal, p_q1):
    quad2_pos = get_object_position(clientID, quad2_handle)
    p_q2 = np.array(quad2_pos)
    path_seg_vec = p_goal - p_q1
    to_q2_vec = p_q2 - p_q1
    seg_norm = np.linalg.norm(path_seg_vec)
    
    proj = np.dot(to_q2_vec, path_seg_vec) / seg_norm
    if proj <= 0.3 or proj >= seg_norm + 0.3:
        time.sleep(0.5)
        return
    elif proj <= 0:
        p_close = p_q1
    elif proj >= seg_norm:
        p_close = p_goal
    else:
        p_close = p_q1 + proj * path_seg_vec / seg_norm

    if np.linalg.norm(p_q2 - p_close) <= 0.3:
        move_target(clientID, target_handle, tuple(p_q1))
        time.sleep(0.25)
        check_path(clientID, target_handle, quad1_handle, quad2_handle, p_goal, p_q1)
    else:
        move_target(clientID, target_handle, tuple(p_goal))
        time.sleep(0.5)
    return

def main():
    # Connect to CoppeliaSim
    sim.simxFinish(-1)
    clientID = sim.simxStart('127.0.0.1', 19997, True, True, 5000, 5)

    if clientID == -1:
        print('Failed to connect to CoppeliaSim')
        return

    print('Connected to CoppeliaSim')

    # Get handle of the helicopter objects and their targets
    _, target1_handle = sim.simxGetObjectHandle(clientID, 'Quadcopter_target', sim.simx_opmode_oneshot_wait)
    _, quad1_handle = sim.simxGetObjectHandle(clientID, 'Quadcopter1', sim.simx_opmode_oneshot_wait)

    _, target2_handle = sim.simxGetObjectHandle(clientID, 'Quadcopter_target#0', sim.simx_opmode_oneshot_wait)
    _, quad2_handle = sim.simxGetObjectHandle(clientID, 'Quadcopter2#0', sim.simx_opmode_oneshot_wait)

    if target1_handle == -1 or quad1_handle == -1 or target2_handle == -1 or quad2_handle == -1:
        print('Failed to get handle of the target object')
        sim.simxFinish(clientID)
        return

    print('Successfully got handles')

    start = (0, -4, 5)
    goal = (0, 4, 5)

    # Create threads for each robot
    thread1 = threading.Thread(target=move_start_goal, args=(clientID, target1_handle, quad1_handle, quad2_handle, start, goal))
    thread2 = threading.Thread(target=move_start_goal, args=(clientID, target2_handle, quad2_handle, quad1_handle, goal, start))


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