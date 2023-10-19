import ros2_numpy
from plyfile import PlyData, PlyElement
import numpy as np
from sensor_msgs.msg import PointCloud2
import rclpy
from box_measurement_interface.srv import BoxDimensionsAndCorners

def main(args=None):

  # ply_file_path = "/home/batman/Downloads/dataset_30/Depth/final_fraction.ply"
  ply_file_path = "/home/batman/Downloads/pizza_box_dataset/pizza_box_0/pizza_box_0_more_trimmed.ply"
  with open(ply_file_path, 'rb') as file:
    ply_data = PlyData.read(file)

  vertices = ply_data['vertex']
  cloud = vertices.data
  
  msg = ros2_numpy.msgify(PointCloud2, cloud)

  # ros side now
  rclpy.init()
  node = rclpy.create_node('service_caller_node')
  client = node.create_client(BoxDimensionsAndCorners, '/box_measurement_node/get_size') 

  while not client.wait_for_service(timeout_sec=1.0):
    node.get_logger().info('service not available, waiting again...')

  # prepare message:
  request = BoxDimensionsAndCorners.Request()
  request.joint_pc = msg

  future = client.call_async(request)
  rclpy.spin_until_future_complete(node, future)

  if future.result() is not None:
    response = future.result()
    node.get_logger().info('Service call succeeded with response: %s' % response.corners)
  else:
    node.get_logger().error('Service call failed')

  node.destroy_node()
  rclpy.shutdown()
