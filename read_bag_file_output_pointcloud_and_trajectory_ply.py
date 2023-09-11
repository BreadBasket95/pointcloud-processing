# import rosbag
# import struct
# import sys
# from sensor_msgs.msg import PointCloud2, PointField
# import sensor_msgs.point_cloud2 as pc2
# from nav_msgs.msg import Odometry
# import numpy as np

# def write_to_ply(points, ply_file):
#     # Write PLY header
#     with open(ply_file, 'wb') as f:
#         f.write(b'ply\n')
#         f.write(b'format binary_little_endian 1.0\n')
#         f.write(b'element vertex %d\n' % len(points))
#         f.write(b'property float x\n')
#         f.write(b'property float y\n')
#         f.write(b'property float z\n')
#         f.write(b'property float sensor_x\n')
#         f.write(b'property float sensor_y\n')
#         f.write(b'property float sensor_z\n')
#         f.write(b'end_header\n')

#         # Write points
#         for x, y, z, sensor_x, sensor_y, sensor_z in points:
#             f.write(struct.pack('ffffff', x, y, z, sensor_x, sensor_y, sensor_z))

# def main(bag_file, ply_file):
#     all_points = []
#     start_time = None
#     for topic, msg, t in rosbag.Bag(bag_file, 'r').read_messages(topics=['/lvi_sam/lidar/mapping/cloud_registered', '/lvi_sam/lidar/mapping/odometry']):
#         if start_time is None:
#             start_time = t

#         elapsed_time = (t - start_time).to_sec()
#         #print("Elapsed time", elapsed_time)
#         if elapsed_time > 10:
#             break

#         if topic == '/lvi_sam/lidar/mapping/cloud_registered':
#             cloud_points = list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))
#         elif topic == '/lvi_sam/lidar/mapping/odometry':
#             sensor_x = msg.pose.pose.position.x
#             sensor_y = msg.pose.pose.position.y
#             sensor_z = msg.pose.pose.position.z

#             if 'cloud_points' in locals():
#                 # Filter out points where x, y, and z are zero
                
#                 points_with_sensor = [
#                     (np.float32(x), np.float32(y), np.float32(z), np.float32(sensor_x), np.float32(sensor_y), np.float32(sensor_z))
#                    for x, y, z in cloud_points if not (np.float32(x) == 0 and np.float32(y) == 0 and np.float32(z) == 0)
#                 ]
#                 all_points.extend(points_with_sensor)
#                 #print("Cloud points float32", all_points)
#     write_to_ply(all_points, ply_file)

# if __name__ == '__main__':
#     print("Checking arguments:", sys.argv)  # Add this before the if statement
#     if len(sys.argv) < 3:
#         print("Usage: python script.py <path_to_bag_file> <path_to_output_ply_file>")
#     else:
#         bag_file = sys.argv[1]
#         ply_file = sys.argv[2]
#         main(bag_file, ply_file)
# import rosbag
# import struct
# import sys
# from sensor_msgs.msg import PointCloud2, PointField
# import sensor_msgs.point_cloud2 as pc2
# from nav_msgs.msg import Odometry
# import numpy as np
# import torch
# from sklearn.neighbors import NearestNeighbors
# import point_cloud_utils as pcu
# # import pcu  # Import the necessary library for point cloud normal estimation

# def write_to_ply_with_normals(points, normals, ply_file):
#     with open(ply_file, 'wb') as f:
#         f.write(b'ply\n')
#         f.write(b'format binary_little_endian 1.0\n')
#         f.write(b'element vertex %d\n' % len(points))
#         f.write(b'property float x\n')
#         f.write(b'property float y\n')
#         f.write(b'property float z\n')
#         f.write(b'property float nx\n')
#         f.write(b'property float ny\n')
#         f.write(b'property float nz\n')
#         f.write(b'end_header\n')

#         for point, normal in zip(points, normals):
#             f.write(struct.pack('ffffff', point[0], point[1], point[2], normal[0], normal[1], normal[2]))

# def normal_func(xyz, sensor):
#     # Your normal calculation logic here, for example:
#     xyz_numpy = xyz.cpu().numpy().astype(np.float32)
#     indices, normal = pcu.estimate_point_cloud_normals_knn(xyz_numpy, 64)
#     normal = torch.from_numpy(normal.astype(np.float32)).to(xyz)
#     indices = torch.from_numpy(indices).to(xyz).long()

#     xyz, sensor = xyz[indices], sensor[indices]
#     non_zero_mask = torch.all(xyz != 0, dim=1)
#     valid_normal_mask = torch.all(normal != 0, dim=1)

#     final_mask = non_zero_mask & valid_normal_mask

#     return xyz[final_mask].cpu().numpy(), normal[final_mask].cpu().numpy()

# def statistical_outlier_removal(xyz, k=20, z_threshold=2.0):
#     nbrs = NearestNeighbors(n_neighbors=k, algorithm='auto').fit(xyz)
#     distances, indices = nbrs.kneighbors(xyz)

#     mean_dist = np.mean(distances, axis=1)
#     threshold = np.mean(mean_dist) + z_threshold * np.std(mean_dist)

#     non_outliers = mean_dist < threshold

#     return xyz[non_outliers]

# def main(bag_file, ply_file):
#     all_points = []
#     all_sensor_positions = []
#     start_time = None

#     for topic, msg, t in rosbag.Bag(bag_file, 'r').read_messages(topics=['/lvi_sam/lidar/mapping/cloud_registered', '/lvi_sam/lidar/mapping/odometry']):
#         if start_time is None:
#             start_time = t

#         elapsed_time = (t - start_time).to_sec()
#         if elapsed_time > 10:
#             break

#         if topic == '/lvi_sam/lidar/mapping/cloud_registered':
#             cloud_points = list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))
#         elif topic == '/lvi_sam/lidar/mapping/odometry':
#             sensor_x = msg.pose.pose.position.x
#             sensor_y = msg.pose.pose.position.y
#             sensor_z = msg.pose.pose.position.z

#             if 'cloud_points' in locals():
#                 points_with_sensor = [
#                     (x, y, z) for x, y, z in cloud_points if not (x == 0 and y == 0 and z == 0)
#                 ]
#                 all_points.extend(points_with_sensor)
#                 all_sensor_positions.extend([(sensor_x, sensor_y, sensor_z)] * len(points_with_sensor))

#     all_points = np.array(all_points, dtype=np.float32)
#     all_sensor_positions = np.array(all_sensor_positions, dtype=np.float32)

#     # Calculate normals
#     xyz = torch.tensor(all_points, dtype=torch.float32).cuda()
#     sensor = torch.tensor(all_sensor_positions, dtype=torch.float32).cuda()
#     filtered_xyz, filtered_normals = normal_func(xyz, sensor)

#     # Perform statistical outlier removal
#     filtered_xyz = statistical_outlier_removal(filtered_xyz)

#     # Write to PLY
#     write_to_ply_with_normals(filtered_xyz, filtered_normals, ply_file)

# if __name__ == '__main__':
#     print("Checking arguments:", sys.argv)
#     if len(sys.argv) < 3:
#         print("Usage: python script.py <path_to_bag_file> <path_to_output_ply_file>")
#     else:
#         bag_file = sys.argv[1]
#         ply_file = sys.argv[2]
#         main(bag_file, ply_file)
import rosbag
import struct
import sys
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2
from nav_msgs.msg import Odometry
import numpy as np
from sklearn.neighbors import NearestNeighbors

def write_to_ply(points, ply_file):
    with open(ply_file, 'wb') as f:
        f.write(b'ply\n')
        f.write(b'format binary_little_endian 1.0\n')
        f.write(b'element vertex %d\n' % len(points))
        f.write(b'property float x\n')
        f.write(b'property float y\n')
        f.write(b'property float z\n')
        f.write(b'property float sensor_x\n')
        f.write(b'property float sensor_y\n')
        f.write(b'property float sensor_z\n')
        f.write(b'end_header\n')
        
        for point in points:
            f.write(struct.pack('ffffff', point[0], point[1], point[2], point[3], point[4], point[5]))

def statistical_outlier_removal(xyz, k=15, z_threshold=2.0):
    nbrs = NearestNeighbors(n_neighbors=k, algorithm='auto').fit(xyz)
    distances, indices = nbrs.kneighbors(xyz)
    mean_dist = np.mean(distances, axis=1)
    threshold = np.mean(mean_dist) + z_threshold * np.std(mean_dist)
    non_outliers = mean_dist < threshold
    return xyz[non_outliers]

def main(bag_file, ply_file):
    all_points = []
    start_time = None

    for topic, msg, t in rosbag.Bag(bag_file, 'r').read_messages(topics=['/lvi_sam/lidar/mapping/cloud_registered', '/lvi_sam/lidar/mapping/odometry']):
        if start_time is None:
            start_time = t

        # uncomment to set the time limit
        #elapsed_time = (t - start_time).to_sec()
        #if elapsed_time > 10:
        #    break

        if topic == '/lvi_sam/lidar/mapping/cloud_registered':
            cloud_points = list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))

        elif topic == '/lvi_sam/lidar/mapping/odometry':
            sensor_x = msg.pose.pose.position.x
            sensor_y = msg.pose.pose.position.y
            sensor_z = msg.pose.pose.position.z

            if 'cloud_points' in locals():
                points_with_sensor = [
                    (x, y, z, sensor_x, sensor_y, sensor_z) for x, y, z in cloud_points if not (x == 0 and y == 0 and z == 0)
                ]
                all_points.extend(points_with_sensor)

    # Convert to NumPy array for easier manipulation
    all_points = np.array(all_points, dtype=np.float32)

    # Perform statistical outlier removal
    filtered_points = statistical_outlier_removal(all_points[:, :3])

    # Associate sensor positions with the filtered points
    filtered_points_with_sensor = np.hstack([filtered_points, all_points[:filtered_points.shape[0], 3:]])
    
    non_zero_mask = np.any(filtered_points_with_sensor[:, :3] != 0, axis=1)
    final_filtered_points_with_sensor = filtered_points_with_sensor[non_zero_mask]

    # Write to PLY file
    write_to_ply(final_filtered_points_with_sensor, ply_file)
    
if __name__ == '__main__':
    print("Checking arguments:", sys.argv)
    if len(sys.argv) < 3:
        print("Usage: python script.py <path_to_bag_file> <path_to_output_ply_file>")
    else:
        bag_file = sys.argv[1]
        ply_file = sys.argv[2]
        main(bag_file, ply_file)