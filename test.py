import time
import sim


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
    _, target_handle = sim.simxGetObjectHandle(clientID, 'Helicopter_target', sim.simx_opmode_oneshot_wait)
    
    if target_handle == -1:
        print('Failed to get handle of the target object')
        sim.simxFinish(clientID)
        return
    
    # Move the helicopter forward and then back
    move_target(clientID, target_handle, [0, -2, 5])

    # Disconnect from CoppeliaSim
    sim.simxFinish(clientID)
    print('Disconnected from CoppeliaSim')

if __name__ == '__main__':
    main()
