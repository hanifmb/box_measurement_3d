import time
import rclpy
import ros2_numpy
import argparse
import numpy as np
from plyfile import PlyData, PlyElement
from sensor_msgs.msg import PointCloud2

def main(args=None):
  parser = argparse.ArgumentParser()
  parser.add_argument('--file', type=str, default='pointcloud.ply', help='location of the point cloud file')
  args = parser.parse_args(args=args)

  ply_file_path = args.file
  with open(ply_file_path, 'rb') as file:
    ply_data = PlyData.read(file)

  vertices = ply_data['vertex']
  cloud = vertices.data

  color = np.vstack((cloud['red'], cloud['green'], cloud['blue']))

  data = np.zeros(cloud.shape[0], dtype=[
    ('x', np.float32),
    ('y', np.float32),
    ('z', np.float32),
    ('vectors', np.float32, (3,))
  ])
  data['x'] = cloud['x']
  data['y'] = cloud['y']
  data['z'] = cloud['z']
  data['vectors'] = np.transpose(color)

  msg = ros2_numpy.msgify(PointCloud2, cloud)

  rclpy.init()
  node = rclpy.create_node('pointcloud_publisher_node')

  publisher = node.create_publisher(PointCloud2, '/camera/depth/color/points', 10)

  print(data)

  while rclpy.ok():
    # node.get_logger().info('Publishing: %s' % msg.data)
    node.get_logger().info('Publishing')
    publisher.publish(msg)
    time.sleep(1) 

  node.destroy_node()
  rclpy.shutdown()

