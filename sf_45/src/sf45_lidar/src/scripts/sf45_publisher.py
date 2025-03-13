#!/usr/bin/env python3
import rospy
import serial
from std_msgs.msg import Float32

# Configure serial port
port = "/dev/ttyACM0"
baudrate = 115200

def read_lidar():
    rospy.init_node('sf45_publisher', anonymous=True)
    pub = rospy.Publisher('/sf45/distance', Float32, queue_size=10)
    rate = rospy.Rate(10)  # 10 Hz

    try:
        ser = serial.Serial(port, baudrate, timeout=1)
        rospy.loginfo("Connected to SF45 LiDAR on {}".format(port))
    except serial.SerialException:
        rospy.logerr("Could not open serial port {}".format(port))
        return

    while not rospy.is_shutdown():
        try:
            line = ser.readline().decode('utf-8').strip()
            if line:
                distance = float(line)  # Assuming SF45 sends distance data in meters
                pub.publish(distance)
                rospy.loginfo("Distance: {:.2f} m".format(distance))
        except Exception as e:
            rospy.logwarn("Error reading from SF45: {}".format(e))

    ser.close()

if __name__ == '__main__':
    try:
        read_lidar()
    except rospy.ROSInterruptException:
        pass
