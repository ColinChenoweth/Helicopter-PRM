import numpy as np
import time
import sim
import PRM
import threading


def move_start_goal(clientID, target_handle, quad_handle, start, goal):
    obs = np.array([[0, 0, 5, 1.75, 1.75, 1.75], [0, 0, 7.5, 0.75, 0.75, 0.75], [0, 0, 2.5, 0.75, 0.75, 0.75], [-2.5, 0, 5, 0.75, 0.75, 0.75], [2.5, 0, 5, 0.75, 0.75, 0.75]])
    
    print('Generating path')
    path = PRM.generate_path(start, goal, 15, obs)

    # for fast testing
    # if goal == (0, -4, 5):
    #     path = [(1.8555338530030134, 1.4652957210390323, 4.230882435623776), (2.753393132968016, 1.5365383838603872, 4.0188706112645), (1.8555338530030134, 1.4652957210390323, 4.230882435623776), (0, -4, 5)]
    # else:
    #     path = [(3.076044060039079, 1.2153788854989962, 2.1726900620985194), (2.685302321070747, 2.925391595515519, 2.0063582194057306), (1.8463993322491055, 3.658439416253085, 3.1699963668226836), (1.0845518634931057, 3.66628265949352, 3.5468239494748026), (0, 4, 5)]
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
    thread1 = threading.Thread(target=move_start_goal, args=(clientID, target1_handle, quad1_handle, start, goal))
    thread2 = threading.Thread(target=move_start_goal, args=(clientID, target2_handle, quad2_handle, goal, start))


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