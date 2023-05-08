import time
import sim


def get_object_bbox_size(clientID, object_handle):
    _, max_x = sim.simxGetObjectFloatParameter(clientID, object_handle, sim.sim_objfloatparam_objbbox_max_x, sim.simx_opmode_blocking)
    _, min_x = sim.simxGetObjectFloatParameter(clientID, object_handle, sim.sim_objfloatparam_objbbox_min_x, sim.simx_opmode_blocking)
    
    _, max_y = sim.simxGetObjectFloatParameter(clientID, object_handle, sim.sim_objfloatparam_objbbox_max_y, sim.simx_opmode_blocking)
    _, min_y = sim.simxGetObjectFloatParameter(clientID, object_handle, sim.sim_objfloatparam_objbbox_min_y, sim.simx_opmode_blocking)
    
    _, max_z = sim.simxGetObjectFloatParameter(clientID, object_handle, sim.sim_objfloatparam_objbbox_max_z, sim.simx_opmode_blocking)
    _, min_z = sim.simxGetObjectFloatParameter(clientID, object_handle, sim.sim_objfloatparam_objbbox_min_z, sim.simx_opmode_blocking)
    
    size_x = max_x - min_x
    size_y = max_y - min_y
    size_z = max_z - min_z
    
    print(size_x, size_y, size_z)


def move_target(clientID, target_handle, target_position):
    print_object_position(clientID, target_handle)
    # Set the position of the target object
    sim.simxSetObjectPosition(clientID, target_handle, -1, target_position, sim.simx_opmode_oneshot)
    print('Moved the target object to the new position')
    print_object_position(clientID, target_handle)



def print_object_position(clientID, object_handle):
    # Get the position of the object
    _, object_position = sim.simxGetObjectPosition(clientID, object_handle, -1, sim.simx_opmode_blocking)
    
    if _ == sim.simx_return_ok:
        print(f"Object position: {object_position}")
    else:
        print("Failed to get object position")


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


    if target_handle == -1:
        print('Failed to get handle of the target object')
        sim.simxFinish(clientID)
        return
    
    # Move the helicopter forward and then back
    move_target(clientID, target_handle, [0, -1, 5])

    get_object_bbox_size(clientID, quad_handle)

    # Disconnect from CoppeliaSim
    sim.simxFinish(clientID)
    print('Disconnected from CoppeliaSim')

if __name__ == '__main__':
    main()
