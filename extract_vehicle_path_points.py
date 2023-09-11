
import struct
import sys
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2
from nav_msgs.msg import Odometry
import numpy as np
from sklearn.neighbors import NearestNeighbors
import time
from pytorch3d.ops import ball_query
import atexit
import point_cloud_utils as pcu
import torch
from tqdm import tqdm



if torch.cuda.is_available(): 
    torch.set_default_tensor_type(torch.cuda.FloatTensor)
    print("Using GPU")
else:
    torch.set_default_tensor_type(torch.FloatTensor)
    print("Using CPU")


def clear_cuda_cache():
    torch.cuda.empty_cache()

atexit.register(clear_cuda_cache)

def write_ply(points, normals, filename):

    with open(filename, 'wb') as f:
        f.write(b'ply\n')
        f.write(b'format binary_little_endian 1.0\n')
        f.write(b'element vertex %d\n' % points.shape[0])
        f.write(b'property float x\n')
        f.write(b'property float y\n')
        f.write(b'property float z\n')
        f.write(b'property float nx\n')
        f.write(b'property float ny\n')
        f.write(b'property float nz\n')
        f.write(b'end_header\n')

        for point, normal in zip(points, normals):
            f.write(struct.pack('ffffff', point[0], point[1], point[2], normal[0], normal[1], normal[2]))

def read_ply(filename):
    with open(filename, 'rb') as f:
        header = b""
        while b"end_header" not in header:
            header += f.readline()
        data = np.fromfile(f, dtype=np.float32)
        data = data.reshape((-1, 6))  # Assuming x, y, z, sensor_x, sensor_y, sensor_z
    return torch.tensor(data, dtype=torch.float32).cuda()  # Directly send to GPU



def filter_points_near_trajectory_grid(xyz, sensor_xyz, radius=3.0):
    # Step 1: Convert the input numpy arrays to PyTorch tensors.
    # We specify the data type as float32 for more efficient calculations.
    # We also move the tensors to the GPU by specifying device='cuda'.
    if isinstance(xyz, np.ndarray):
        xyz = torch.tensor(xyz, dtype=torch.float32, device='cuda')
    else:
        xyz = xyz.clone().detach().to(dtype=torch.float32, device='cuda')

    sensor_xyz = torch.tensor(sensor_xyz, dtype=torch.float32, device='cuda')
    
    # Display the shapes of the tensors for debugging.
    print(f"xyz shape: {xyz.shape}")
    print(f"sensor_xyz shape: {sensor_xyz.shape}")
    
    # Step 2: Remove duplicate sensor points.
    # We use torch.unique to get a tensor containing unique rows from sensor_xyz.
    unique_sensor_xyz = torch.unique(sensor_xyz, dim=0)
    
    # Display the shape of unique_sensor_xyz for debugging.
    print(f"unique_sensor_xyz shape: {unique_sensor_xyz.shape}")
    
    # Step 3: Initialize sets to keep track of indices.
    # near_indices will contain indices of points in xyz that are within the radius of any sensor point.
    # checked_indices will contain indices of points in xyz that we've already checked.
    near_indices = set()
    overall_mask = torch.zeros(xyz.shape[0], dtype=torch.bool, device='cuda')
    checked_indices = set()
    # Convert the radius to a float32 tensor on the GPU for efficient computations.
    radius = torch.tensor([radius], dtype=torch.float32, device='cuda')

    # pbar = tqdm(total=len(unique_sensor_xyz), desc='maskin point cloud down to those within radius of any sensor point', unit='sensor_pt')

    # for sensor_point in unique_sensor_xyz:
    #     mask_x = (xyz[:, 0] >= sensor_point[0] - radius) & (xyz[:, 0] <= sensor_point[0] + radius)
    #     mask_y = (xyz[:, 1] >= sensor_point[1] - radius) & (xyz[:, 1] <= sensor_point[1] + radius)
    #     mask = mask_x & mask_y
    #     overall_mask |= mask  # Update the overall mask
    #     pbar.update(1)
    # pbar.close()
    # print("applying mask we found to xyz")

    #filtered_xyz = xyz[overall_mask]

    #return filtered_xyz.cpu().numpy(), list(near_indices)


    pbar = tqdm(total=len(unique_sensor_xyz), desc='Processing sensor points', unit='sensor_pt')
    # Step 4: Loop over each unique sensor point to find nearby points.

    for sensor_point in unique_sensor_xyz:
        # Create boolean masks for points that are within the bounding box around the sensor point.
        # mask_x = (filtered_xyz[:, 0] >= sensor_point[0] - radius) & (filtered_xyz[:, 0] <= sensor_point[0] + radius)
        # mask_y = (filtered_xyz[:, 1] >= sensor_point[1] - radius) & (filtered_xyz[:, 1] <= sensor_point[1] + radius)
        mask_x = (xyz[:, 0] >= sensor_point[0] - radius) & (xyz[:, 0] <= sensor_point[0] + radius)
        mask_y = (xyz[:, 1] >= sensor_point[1] - radius) & (xyz[:, 1] <= sensor_point[1] + radius)
        mask_z = (xyz[:, 2] >= sensor_point[2] - radius +1) & (xyz[:, 1] <= sensor_point[1] + radius-1)

        # Combine the x and y masks to get a single mask.
        mask = mask_x & mask_y & mask_z
        
        #point_xyz = filtered_xyz[mask]
        #print("point_xyz" , point_xyz.shape)
        # Use the mask to get the indices of points within the bounding box.
        bounding_box_indices = torch.where(mask)[0]
        
        # Convert these indices to CPU and then to numpy format for set operations.
        bounding_box_indices_cpu = bounding_box_indices.cpu().numpy()
        #print("bounding_box_indices_cpu" , bounding_box_indices_cpu.shape)
        # Remove indices of points that we've already checked.
        new_points = set(bounding_box_indices_cpu)# - checked_indices
        
        # Step 5: Check the distance from the sensor point to each point within the bounding box.
        for idx in new_points:
            near_indices.add(idx.item())
            
            #point = filtered_xyz[idx, :2]
            #distance = torch.linalg.norm(sensor_point[:2] - point)
            
            # If the point is within the radius, add its index to near_indices.
            #if distance <= radius:
            #    near_indices.add(idx.item())  # Convert to Python int for set operations.
    
     # Update checked_indices with the indices of points we've just checked.
        checked_indices.update(new_points)
        
        pbar.update(1)
    
    pbar.close()

    # Step 6: Create a new tensor containing only the points that are within the radius of any sensor point.
    final_filtered_xyz = xyz[list(near_indices)]

    # Display the shape of the final tensor for debugging.
    print(f"Final filtered xyz shape: {final_filtered_xyz.shape}")
    
    # Convert the final tensor to numpy format and move it to CPU.
    return final_filtered_xyz.cpu().numpy(), list(near_indices)


def normal_func(xyz, sensor):
    start_time = time.time()

    if isinstance(xyz, np.ndarray):
        xyz = torch.tensor(xyz, dtype=torch.float32, device='cuda')
    if isinstance(sensor, np.ndarray):
        sensor = torch.tensor(sensor, dtype=torch.float32, device='cuda')

    xyz_numpy = xyz.cpu().numpy().astype(np.float32)
    indices, normal = pcu.estimate_point_cloud_normals_knn(xyz_numpy, 64)
    normal = torch.tensor(normal.astype(np.float32)).cuda()
    xyz, sensor = xyz[indices], sensor[indices]

    non_zero_mask = torch.any(xyz != 0, dim=1)
    xyz, normal = xyz[non_zero_mask], normal[non_zero_mask]

    view_dir = sensor - xyz
    view_dir = view_dir / (torch.linalg.norm(view_dir, dim=-1, keepdim=True) + 1e-6)
    cos_angle = torch.sum(view_dir * normal, dim=1)
    cos_mask = cos_angle < 0.0
    #normal[cos_mask] = -normal[cos_mask]
    normal[cos_mask].neg_()

    keep_mask = torch.abs(cos_angle) > np.cos(np.deg2rad(85.0))
    xyz, normal = xyz[keep_mask], normal[keep_mask]

    clear_cuda_cache()

    print(f"Time taken for normal_func: {time.time() - start_time} seconds")

    return xyz, normal



def main(input_ply, filtered_ply):
    # Read PLY data
    data = read_ply(input_ply)
    
    # Convert the data to PyTorch tensors and move them to the GPU
    data = data.clone().detach().to(dtype=torch.float32, device='cuda')    

    # Separate point cloud and sensor positions
    xyz = data[:, :3]
    sensors = data[:, 3:]
    
    # Filter out zeros and points near the trajectory
    non_zero_mask = torch.any(xyz != 0, dim=1)
    xyz = xyz[non_zero_mask]
    sensors = sensors[non_zero_mask]
    
    # calculate normals before segmenting down to the points near the trajectory because otherwise it filters out good points
    #filtered_xyz, filtered_normals = normal_func(filtered_xyz, sensors[:filtered_xyz.shape[0], :])
    filtered_xyz, filtered_normals = normal_func(xyz, sensors[:xyz.shape[0], :])
    #filtered_data = np.hstack([filtered_xyz, filtered_normals[:filtered_xyz.shape[0], :]])

    

    # Filtering points near the trajectory
    filtered_xyz_with_normals, indices_used = filter_points_near_trajectory_grid(filtered_xyz, sensors)
    #print("indices_used", indices_used)
    filtered_normals = filtered_normals[indices_used, :]
    # Convert filtered data back to CPU and numpy for writing to PLY
    #filtered_xyz = filtered_xyz.cpu().numpy()
    #filtered_normals = filtered_normals.cpu().numpy()
    
    # Write to PLY
    write_ply(filtered_xyz_with_normals, filtered_normals, "brownfield_no_z.ply")

if __name__ == '__main__':
    try:
        input_ply = "idk.ply"  # Replace with your input PLY file
        filtered_ply = "idk_filtered_trajectories.ply"  # Replace with your desired output PLY file for filtered points
        main(input_ply, filtered_ply)
    except KeyboardInterrupt:
        print("Program interrupted. Cleaning up...")
    finally:
        clear_cuda_cache()