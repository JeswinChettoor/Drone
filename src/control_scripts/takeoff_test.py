import time
from pymavlink import mavutil

def create_connection(conn_id):
    connection = mavutil.mavlink_connection(conn_id)
    print("Waiting for heartbeat...")
    connection.wait_heartbeat()
    print("Connected to drone")
    print("Requesting position")
    connection.mav.command_long_send(
        connection.target_system,
        connection.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
        0,
        mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED,
        50000,
        0, 0, 0, 0, 0,
    )
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

def takeoff(connection,alt=0):
    connection.mav.command_long_send(
        connection.target_system,
        connection.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0,         
        0,          
        0, 0,       
        0,          
        0, 0,       
        alt
    )
    print("Told drone to takeoff")

def get_z(connection, timeout=2):
    msg = connection.recv_match(type='LOCAL_POSITION_NED', blocking=True, timeout=timeout)
    return msg.z if msg else None

def reachedalt(connection,alt):
    start_z=get_z(connection)
    start_time=time.time()
    while time.time()-start_time<10:
        msg = connection.recv_match(type='LOCAL_POSITION_NED', blocking=True, timeout=1)
        if msg:
            climbed=start_z-msg.z
            if abs(alt - climbed) <= 0.3:
                # Change accuracy later
                print("Reached altitude")
                return True
    print("Failed to reach altitude")
    return False


def takeoffandland(connection,alt):
    change_mode(connection,'GUIDED')
    arm_disarm(connection,arm=True)
    takeoff(connection,alt)
    if reachedalt(connection,alt):
        print("Hold for 5 seconds")
        time.sleep(5)
        change_mode(connection,'LAND')
        time.sleep(15)
        #have to change to check
        arm_disarm(connection,arm=False)

    else:
        print("Didnt reacch alt")
        change_mode(connection,"LAND")
    


if __name__ == "__main__":
    drone = create_connection('udp:127.0.0.1:15001')
    #Change connection port
    takeoffandland(drone,alt=2)


    
