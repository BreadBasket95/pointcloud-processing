#!/usr/bin/env python

import rosbag
import os

# Initialize an empty list to hold the x, y, z positions
positions = []

# Open the rosbag file
with rosbag.Bag('2023-08-30-10-43-38.bag', 'r') as bag:
    for topic, msg, t in bag.read_messages(topics=['/lvi_sam/lidar/mapping/odometry']):
        # Extract the x, y, z positions from the odometry message
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        z = msg.pose.pose.position.z

        # Append the positions to the list
        positions.append((x, y, z))

# Save the positions to a text file
with open('sensor_positions.txt', 'w') as f:
    for x, y, z in positions:
        f.write(f"{x} {y} {z}\n")