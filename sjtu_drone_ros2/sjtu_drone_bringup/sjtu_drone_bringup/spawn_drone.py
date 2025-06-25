#!/usr/bin/env python3
import sys
import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SpawnEntity
from geometry_msgs.msg import Pose


def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node('spawn_drone')
    cli = node.create_client(SpawnEntity, '/spawn_entity')

    if len(sys.argv) < 6:
        node.get_logger().error('Usage: spawn_drone <robot_desc> <namespace> <x> <y> <z>')
        rclpy.shutdown()
        return

    content = sys.argv[1]
    namespace = sys.argv[2]
    x = float(sys.argv[3])
    y = float(sys.argv[4])
    z = float(sys.argv[5])

    req = SpawnEntity.Request()
    req.name = namespace
    req.xml = content
    req.robot_namespace = namespace
    req.reference_frame = "world"

    req.initial_pose = Pose()
    req.initial_pose.position.x = x
    req.initial_pose.position.y = y
    req.initial_pose.position.z = z
    req.initial_pose.orientation.w = 1.0  # No rotation

    while not cli.wait_for_service(timeout_sec=1.0):
        node.get_logger().info('Waiting for /spawn_entity service...')

    future = cli.call_async(req)
    rclpy.spin_until_future_complete(node, future)

    if future.result() is not None:
        node.get_logger().info(
            'Spawn success: ' + str(future.result().success) + " | " + future.result().status_message)
    else:
        node.get_logger().info('Service call failed: %r' % (future.exception(),))

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
