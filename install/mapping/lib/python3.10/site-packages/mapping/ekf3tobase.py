"Takes orientation from ekf3 and height and gives the transform base_footprint to base_link"
import rclpy 
from pymavlink import mavutil
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from tf_transformations import quaternion_from_euler

def connect_mavlink():
    interval = 5000
    # Time interval between messages . IN MICROSECONDS
    connection=mavutil.mavlink_connection('udpin:127.0.0.1:15000')
    #This is when it is connected via udp . When connected via usb , set string to /dev/ttyUSB0
    connection.wait_heartbeat(timeout=30)
    print("Ardupilot bridge : Connected to Ardupilot")
    message_ids=[32,30]
    #32 is for local_position_ned , 30 for attitude
    for message_id in message_ids:
        message = connection.mav.command_long_encode(
        connection.target_system,
        connection.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
        0,
        message_id,
        interval,0,0,0,0,0)
        connection.mav.send(message)
    response = connection.recv_match(type='COMMAND_ACK', blocking=True)
    if response and response.command == mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL and response.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
        print("Setting stream rate of position_local_ned and attitude successful !")
    else:
        print("Unable to set position_ned and attitude streaming rates")
    return connection

def transform_publisher(node, broadcaster, height, roll, pitch):
    z = height  
    transform = TransformStamped()
    transform.header.stamp = node.get_clock().now().to_msg()
    transform.header.frame_id = 'base_footprint'
    transform.child_frame_id = 'base_link'
    transform.transform.translation.x = 0.0
    transform.transform.translation.y = 0.0
    transform.transform.translation.z = z
    qx, qy, qz, qw = quaternion_from_euler(roll, pitch, 0.0)
    transform.transform.rotation.x = qx
    transform.transform.rotation.y = qy
    transform.transform.rotation.z = qz
    transform.transform.rotation.w = qw
    broadcaster.sendTransform(transform)
    #print("Transform base_footprint to base_link published")
    
def main():
    connection = connect_mavlink()
    rclpy.init()
    node = rclpy.create_node('footprint2link')
    broadcaster = TransformBroadcaster(node)

    z = 0.0
    roll = None
    pitch = None

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.0)
            receivedmsg = connection.recv_match(
                type=['LOCAL_POSITION_NED', 'ATTITUDE'],
                blocking=True,
                timeout=1
            )

            if receivedmsg is None:
                continue

            msg_type = receivedmsg.get_type()
            #print("Message stream running")

            if msg_type == 'ATTITUDE':
                # print("Attitude message running")
                pitch = -receivedmsg.pitch
                roll = receivedmsg.roll
                #print('getting roll,pitch')

            elif msg_type == 'LOCAL_POSITION_NED':
                #print('local position ned message running')
                z = -receivedmsg.z

            # Publish whenever attitude is available
            if roll is not None and pitch is not None:
                transform_publisher(
                    node, broadcaster, z, roll, pitch
                )
                #print("Message sent to publisher")

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
