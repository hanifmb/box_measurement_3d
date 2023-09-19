import rclpy
import ros2_numpy
import numpy as np
from box_measurement_interface.srv import BoxDimensionsAndCorners
from plyfile import PlyData, PlyElement
from geometry_msgs.msg import Point32
from itertools import product

class Plane():
  def __init__(self, coeff=[]):
    self.coeff = coeff
    self.normal = coeff[:3]
    self.d = coeff[-1]
    self.inlier_indices = []

def fitPlane(three_points):
  v1 = three_points[1] - three_points[0]
  v2 = three_points[2] - three_points[0]
  normal = np.cross(v1, v2)
  normal /= np.linalg.norm(normal)
  d = -np.dot(normal, three_points[0])
  return [normal[0], normal[1], normal[2], d]

def seq_ransac(cloud, threshold=0.005, n_planes=5):
  def ransac_plane_fit(points, n_iters):
    best_plane = []
    best_inliers = []
    max_inliers = 0

    for i in range(n_iters):
      random_indices = np.random.choice(len(points), 3, replace=False) # select three points randomly
      sample_points = points[random_indices]
      # plane = Plane(sample_points) # fit a plane 
      plane_coeff = fitPlane(sample_points)
      plane = Plane(plane_coeff)

      distances = np.abs(np.dot(points - sample_points[0], plane.normal)) # calc distance
      inliers = np.where(distances < threshold)[0]
      num_inliers = len(inliers) # calc num of inliers
      # Update the best plane if necessary
      if num_inliers > max_inliers:
        best_plane = plane
        best_plane.inlier_indices = inliers
        max_inliers = num_inliers
    return best_plane

  n_points = len(cloud)
  points_indices = np.arange(n_points)
  remaining_points = cloud

  final_planes = []
  final_inliers = []
  for i in range(n_planes):
    # roughly calculate num of iterations (N)
    # p = 0.99; s = 3; num_inliers = 45000
    # eps = (len(remaining_points) - num_inliers) / len(remaining_points)
    # N = round(np.log(1-p) / np.log(1-(1-eps)**s)) 
    # N = 1000 if N > 1000 else N
    N = 1000
    plane = ransac_plane_fit(remaining_points, N) # run ransac
    # update the inlier to the global indices after removing
    local_points_indices = plane.inlier_indices.copy()
    plane.inlier_indices[:] = points_indices[local_points_indices]
    final_planes.append(plane)
    # remove the inliers
    mask = np.ones(len(remaining_points), dtype=bool)
    mask[local_points_indices] = False
    remaining_points = remaining_points[mask]
    # remove the original points indices belong to inliers
    mask2 = np.ones(len(points_indices), dtype=bool)
    mask2[local_points_indices] = False
    points_indices = points_indices[mask2]
  return final_planes

def export_planes(points, planes):
  length = len(points)
  rgb = np.full((length, 3), (255, 255, 255)) 

  rgb[planes[0].inlier_indices] = (255, 0, 0)
  rgb[planes[1].inlier_indices] = (0, 255, 0)
  rgb[planes[2].inlier_indices] = (0, 0, 255)
  rgb[planes[3].inlier_indices] = (255, 255, 0)
  rgb[planes[4].inlier_indices] = (0, 255, 255)

  rgb_np = np.array(rgb)
  ver = np.hstack((points, rgb_np))
  new_ver = np.core.records.fromarrays(ver.transpose(), 
                                       names='x, y, z, red, green, blue',
                                       formats = 'f4, f4, f4, u1, u1, u1')
  el = PlyElement.describe(new_ver, 'vertex')
  PlyData([el], text=True).write('/home/batman/Downloads/dataset_30/ascii.ply')

def get_planes_angle(plane1_coefficients, plane2_coefficients):
  normal_vector1 = np.array(plane1_coefficients[:3])
  normal_vector2 = np.array(plane2_coefficients[:3])

  dot_product = np.dot(normal_vector1, normal_vector2)

  magnitude1 = np.linalg.norm(normal_vector1)
  magnitude2 = np.linalg.norm(normal_vector2)

  angle_radians = np.arccos(dot_product / (magnitude1 * magnitude2))

  if angle_radians > np.pi / 2:
    angle_radians = np.pi - angle_radians

  angle_degrees = np.degrees(angle_radians)

  return angle_degrees

def handle_request(request, response):
  print("request received")
  pointcloud = ros2_numpy.numpify(request.joint_pc)
  pointcloud_np = np.array(pointcloud.tolist())
  points_np = pointcloud_np[:, :3]

  planes = seq_ransac(points_np)
  export_planes(points_np, planes) 

  red_plane = Plane([-0.003401960104850783, 0.007624634496218392, -0.9999651452006935, 0.032146114540822605]) # base plane
  # green_plane = Plane([0.9999861459093223, -0.0019003673037874186, -0.0049088281218811584, -0.29748315839140277])
  # blue_plane = Plane([0.020923692537118403, 0.999780547287266, 0.001027792093321174, -0.936057456637583])

  ANGLE_THRESHOLD = 5.0

  # find plane pairs for the width and length side
  planes_pair = []
  for i in range(len(planes)):
    for j in range(i + 1, len(planes)):
      # angle between box sides 
      angle = get_planes_angle(planes[i].normal, planes[j].normal)
      if angle < ANGLE_THRESHOLD:
        planes_pair.append((planes[i], planes[j]))

  if len(planes_pair) != 2: print("Side plane pair(s) is not found!"); return

  # find the plane pair for top and bottom sides
  top_plane = None
  for plane in planes:
    angle = get_planes_angle(plane.normal, red_plane.normal)
    if angle < ANGLE_THRESHOLD:
      top_plane = plane

  if top_plane is None: print("Top plane pair is not found!"); return
  planes_pair.append((top_plane, red_plane))

  # combinations of three
  corners = []
  combinations = list(product(planes_pair[0], planes_pair[1], planes_pair[2]))
  filtered_combinations = [combo for combo in combinations if len(set(combo)) == 3]
  for combo in filtered_combinations:
    corner = np.linalg.solve(np.array([combo[i].normal for i in range(3)]), -1*np.array([combo[i].d for i in range(3)]))

    point = Point32()
    point.x = corner[0]
    point.y = corner[1]
    point.z = corner[2]
    corners.append(point)

  sorted_corner_x = sorted(corners, key=lambda point: point.x)
  sorted_corner_y = sorted(corners, key=lambda point: point.y)
  sorted_corner_z = sorted(corners, key=lambda point: point.z)

  def corner_mean(sorted_corner):
    closer_plane = sorted_corner[:4]
    farther_plane = sorted_corner[-4:]

    closer_plane_np = np.array([(point.x, point.y, point.z) for point in closer_plane])
    farther_plane_np = np.array([(point.x, point.y, point.z) for point in farther_plane])

    average_point_closer = np.mean(closer_plane_np, axis=0)
    average_point_farther = np.mean(farther_plane_np, axis=0)

    distance = np.linalg.norm(average_point_closer - average_point_farther)
    print(distance)

  w = corner_mean(sorted_corner_x)
  l = corner_mean(sorted_corner_y)
  h = corner_mean(sorted_corner_z)

  print(w, l, h)

  # Calculate the average point using NumPy
  response.corners = corners
  response.size = [w, l, h]

  return response

def main(args=None):
  rclpy.init(args=args)
  node = rclpy.create_node('box_measurement_node')
  service = node.create_service(BoxDimensionsAndCorners, '/box_measurement_node/get_size', handle_request)

  print('Service server is ready to receive requests.')

  rclpy.spin(node)
  node.destroy_node()
  rclpy.shutdown()
