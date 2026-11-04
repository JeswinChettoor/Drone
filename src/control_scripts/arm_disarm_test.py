import time
from pymavlink import mavutil

def create_connection(conn_id):
    connection = mavutil.mavlink_connection(conn_id)
    print("Waiting for heartbeat...")
    connection.wait_heartbeat()
    print("Connected to system")
    return connection

def change_mode(connection,mode):
    mode_map=connection.mode_mapping()
    if not mode_map or mode not in mode_map:
            print("Unknown mode , error")
            return False
    mode_id=mode_map[mode]
    print("Desired mode: ",mode)
    print("Desired Mode ID :",mode_id)
    print("Attempting to change mode")
    connection.mav.set_mode_send(connection.target_system,mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,mode_id)

    start_time = time.time()
    while time.time() - start_time < 5 :
        msg=connection.recv_match(type='HEARTBEAT',blocking=False)
        if msg and msg.custom_mode == mode_id:
            print("Mode change successful")
            return True
        time.sleep(0.1)
    print("Didn't happen")
    return False

def arm_disarm(connection, arm=True, timeout=5):
    print("Attempting to arm" if arm else "Attempting to disarm")

    connection.mav.command_long_send(
        connection.target_system,
        connection.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,  
        0,      
        1 if arm else 0,   
        0, 0, 0, 0, 0, 0,   
    )

    start_time = time.time()
    while time.time() - start_time < timeout:
        msg = connection.recv_match(type='COMMAND_ACK', blocking=True, timeout=1)
        if msg and msg.command == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM:
            if msg.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                print("Arm/disarm successful")
                return True
            print("Arm/disarm rejected")
            return False
    print("Arm/disarm timed out")
    return False

def armthendisarm(connection):
    if not change_mode(connection, 'STABILIZE'):
        return False
    print("Drone will arm, then in 5 seconds, disarm")
    if not arm_disarm(connection, arm=True):
        return False
    time.sleep(5)
    if arm_disarm(connection, arm=False):
        print("Drone disarmed")
        return True
    return False
if __name__ == "__main__":
    drone = create_connection('udp:127.0.0.1:15001')
    armthendisarm(drone)

    
