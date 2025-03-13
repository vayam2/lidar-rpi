#!/usr/bin/env python3
import rospy
import numpy as np
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from collections import deque
import threading

# Data buffer to handle high-frequency updates
data_buffer = deque(maxlen=10)  # Stores the last 10 scans
buffer_lock = threading.Lock()
scan_direction = 1  # Tracks scan direction: 1 = forward, -1 = reverse


def normalize_angle(angle_degrees):
    """Convert SF45 LiDAR angle to Matplotlib polar coordinates (0 to 2π radians)."""
    global scan_direction
    if angle_degrees > 160:
        scan_direction = -1  # Reverse scan
    elif angle_degrees < -160:
        scan_direction = 1  # Forward scan
    
    if scan_direction == -1:
        angle_degrees = 320 - angle_degrees  # Mirror reverse scan
    
    return angle_degrees  # Keep in degrees since we're printing


def callback(data):
    """Callback function to process PointCloud2 data and print sector values."""
    global data_buffer
    points = list(pc2.read_points(data, field_names=("x", "y", "z"), skip_nans=True))

    if len(points) == 0:
        rospy.logwarn("No points received from SF45 LiDAR")
        return

    # Extract x, y points
    raw_angles = np.arctan2([p[1] for p in points], [p[0] for p in points]) * 180 / np.pi  # Convert to degrees
    distances = np.sqrt(np.array([p[0] for p in points]) ** 2 + np.array([p[1] for p in points]) ** 2)  # Distances

    # Normalize angles
    corrected_angles = np.array([normalize_angle(a) for a in raw_angles])

    # Filter out only the incorrect 1.00m values
    filtered_data = [(corrected_angles[i], distances[i]) for i in range(len(corrected_angles)) if distances[i] != 1.00]

    # Print filtered sector values in terminal
    if filtered_data:
        rospy.loginfo("\n---- SF45 LiDAR Sector Readings ----")
        for angle, distance in filtered_data:
            if not 0.99 < distance < 1.01:

                rospy.loginfo(f"Angle: {angle:.1f}°, Distance: {distance:.2f}m")

    # Append new filtered data to buffer safely
    with buffer_lock:
        data_buffer.append(filtered_data)


def listener():
    """ROS Node to subscribe to SF45 LiDAR topic and print sector values."""
    rospy.init_node('sf45_print_values_filtered', anonymous=True)
    rospy.Subscriber("/pointcloud", PointCloud2, callback, queue_size=1)
    rospy.spin()  # Keep the node alive


if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
