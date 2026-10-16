'''Goal of this file is to subscribe to mavlink streams , then publish position from ekf3 as odometry . Also acts as a router for all 
info that needs to be taken from ardupilot . As of now , going with the old approach .'''
import math
import time
import rclpy
from pymavlink import mavutil
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

def connect_mavlink():
    interval = 60000
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

def transform_publisher(node,broadcaster,position,yaw):
    x, y = position.x, -position.y
    qz, qw = math.sin(yaw / 2.0), math.cos(yaw / 2.0)     
    now = node.get_clock().now().to_msg()
    transform = TransformStamped()
    transform.header.stamp = now
    transform.header.frame_id = 'odom'
    transform.child_frame_id = 'base_footprint'
    transform.transform.translation.x = x
    transform.transform.translation.y = y
    transform.transform.rotation.z = qz
    transform.transform.rotation.w = qw
    broadcaster.sendTransform(transform)
    #print("Transform odom to base_link published")
def main():
    connection=connect_mavlink()
    rclpy.init()
    node=rclpy.create_node('ardupilotBridge')
    broadcaster = TransformBroadcaster(node)
    yaw=None

    try:
        while True:
            receivedmsg=connection.recv_match(type=['LOCAL_POSITION_NED', 'ATTITUDE'], blocking=True,timeout=1)
            if receivedmsg is None:
                continue
            msg_type = receivedmsg.get_type()
            #print("Message stream running")
            if msg_type == 'ATTITUDE':
               # print("Attitude message running")
                yaw = -receivedmsg.yaw
                #print('getting yaw')
            elif msg_type == 'LOCAL_POSITION_NED' and yaw is not None :
                #print('local position ned message running')
                transform_publisher(node, broadcaster, receivedmsg, yaw)
                #print("Message sent to publisher")

    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()



if __name__ == "__main__":
    main()