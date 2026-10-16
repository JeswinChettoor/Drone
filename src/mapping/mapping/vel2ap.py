'''This file takes /cmd_vel commands from  nav2 , sends this to mavlink'''

import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from pymavlink import mavutil

def connect_mavlink():
    connection=mavutil.mavlink_connection('udpin:127.0.0.1:14551')
    #This is when it is connected via udp . When connected via usb , set string to /dev/ttyUSB0
    connection.wait_heartbeat(timeout=30)
    print("Ardupilot bridge : connected to AP : ready to send cmd_vel to AP")
    return connection
def sendvelocitytarget(vx,vy,vz,yaw_rate,connection):
    # Ignore position (bits 0-2), acceleration (bits 6-8),
    # and yaw angle (bit 10).
    # Keep velocity (bits 3-5) and yaw rate (bit 11) active.
    type_mask = 1479
    #VERIFY THIS  LATER
    #9: FRAME BODY OFFSET NED

    connection.mav.set_position_target_local_ned_send(0,connection.target_system,
                                                      connection.target_component,
                                                      9,type_mask,
                                                      0.0,0.0,0.0 ,#ignored position
                                                      vx,vy,vz,
                                                      0.0,0.0,0.0, #acceleration , ignored
                                                      0.0, #yaw_angle
                                                      yaw_rate
                                    )               
    
class VELTOAP(Node):
    def __init__(self):
        super().__init__('VELTOAP')
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.connection=connect_mavlink()
        self.vx = 0.0
        self.vy = 0.0
        self.vz = 0.0
        self.yaw_rate = 0.0
    def listener_callback(self, msg):
        self.get_logger().info('Got message from cmd_vel')
        self.vx=msg.linear.x
        self.vy=-msg.linear.y
        self.vz=0
        self.yaw_rate=-msg.angular.z
        sendvelocitytarget(self.vx,self.vy,self.vz,self.yaw_rate,self.connection)

def main():
    rclpy.init()
    node = None
    try:
        node = VELTOAP()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None: node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
if __name__ == "__main__":
    main()
