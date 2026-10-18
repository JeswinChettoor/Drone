import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseArray, PoseStamped

input_topic = '/localized_person'
output_topic = '/confirmed_survivors'

person_prox = 2.0 #Radius within which it is considered to be the same person
min_count = 20
#Minimum number of detections needed to qualify as a cluster

class SurvivorEvidence(Node):
    def __init__(self):
        super().__init__("survivor_confirmation_node")
        self.clusters = []   # {'points': [[x, y], ...], 'confirmed': True/False}
        self.pub = self.create_publisher(PoseArray, output_topic, 10)
        self.create_subscription(PoseStamped, input_topic, self.on_observation, 10)
        self.create_timer(1.0, self.publish_confirmed)

    def on_observation(self, msg):
        x = msg.pose.position.x
        y = msg.pose.position.y

        cluster = self.find_cluster(x, y)
        if cluster is None: 
            self.clusters.append({'points': [[x, y]], 'confirmed': False})
        else:
            # Add point
            cluster['points'].append([x, y])

            if len(cluster['points']) >= min_count and not cluster['confirmed']:
                cluster['confirmed'] = True
                avg_x, avg_y = self.average(cluster)
                self.get_logger().info("CONFIRMED survivor at x=" + str(avg_x) +
                                       " y=" + str(avg_y))

    def find_cluster(self, x, y):
        for cluster in self.clusters:
            avg_x, avg_y = self.average(cluster)
            distance = math.sqrt((avg_x - x) ** 2 + (avg_y - y) ** 2)
            if distance <= person_prox:
                return cluster
        return None

    def average(self, cluster):
        total_x = 0.0
        total_y = 0.0
        for point in cluster['points']:
            total_x = total_x + point[0]
            total_y = total_y + point[1]
        count = len(cluster['points'])
        return total_x / count, total_y / count

    def publish_confirmed(self):
        out = PoseArray()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = "map"

        for cluster in self.clusters:
            if cluster['confirmed']:
                avg_x, avg_y = self.average(cluster)
                pose = Pose()
                pose.position.x = avg_x
                pose.position.y = avg_y
                pose.position.z = 0.0
                pose.orientation.w = 1.0
                out.poses.append(pose)

        self.pub.publish(out)

def main():
    rclpy.init()
    node = SurvivorEvidence()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
