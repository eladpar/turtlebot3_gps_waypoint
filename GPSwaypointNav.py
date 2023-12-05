#!/usr/bin/env python

import rospy
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Twist
from geographic_msgs.msg import GeoPoint
from math import atan2, radians, degrees, pi
from nav_msgs.msg import Odometry
import pyproj
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import tf
class GPSFollower:
    def __init__(self):
        rospy.init_node('gps_follower_node', anonymous=True)

        # Subscribe to the /fix topic to get GPS data
        rospy.Subscriber('/robot_location', NavSatFix, self.gps_callback)
        rospy.Subscriber('/target_location', GeoPoint, self.target_callback)
        rospy.Subscriber ('odom', Odometry, self.get_rotation)
        # Create a publisher for sending velocity commands
        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        
        # Get latitude and longitude parameters
        # 300 meter away with bearing
        # self.target_latitude = rospy.get_param('~target_latitude', 32.073396303370245)
        # self.target_longitude = rospy.get_param('~target_longitude', 34.78357231127559)
        #40 meter away
        self.target_latitude = rospy.get_param('~target_latitude', 32.072617848159176)
        self.target_longitude = rospy.get_param('~target_longitude', 34.787039205658836)

        # Tolerance for considering reaching the target coordinates (in meters)
        self.tolerance = 2.0
        self.roll = self.pitch = self.yaw = 0.0
        self.target = 90
        self.kp=0.5
        self.once = False
        self.start = False
        # Initialize variables for keeping track of last few angle differences
        self.last_few_angle_diff = []
        self.max_past_angles = 5  # Adjust this as needed
        self.prev_error_angle = 0.0
        self.integral_angle = 0.0
        self.dt = 0.1  # Time step for PID

        # PID constants for angle control
        self.kp_angle = 0.2
        self.ki_angle = 0.0
        self.kd_angle = 0.05

        # Maximum angular velocity
        self.max_angular_vel = 0.2
    def gps_callback(self, data):
        current_latitude = data.latitude
        current_longitude = data.longitude
        if self.start == True:
            geodesic = pyproj.Geod(ellps='WGS84')
            _,gps_angle,distance = geodesic.inv(current_longitude, current_latitude, self.target_longitude, self.target_latitude)
          # If the robot hasn't reached the target coordinates yet
            if distance > self.tolerance:
                # Calculate the angle between the current position and the target coordinates

                angle_to_target = 180 - gps_angle
                move_cmd = self.generate_move_command(angle_to_target)
                rospy.loginfo(f"Distance: {distance} | gpsangle {gps_angle} | angle to target: {angle_to_target} | yaw {degrees(self.yaw)}| differnce in angle is: {degrees(radians(angle_to_target) - self.yaw)}")

                self.vel_pub.publish(move_cmd)
            else:
                # Stop the robot when it reaches the target coordinates
                stop_cmd = Twist()
                self.vel_pub.publish(stop_cmd)
                rospy.loginfo("Target coordinates reached!")

    def target_callback(self, data):  
        self.target_latitude = data.latitude
        self.target_longitude = data.longitude
        self.start = True
    def get_rotation (self , msg):
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (self.roll, self.pitch, self.yaw) = euler_from_quaternion (orientation_list)

    def generate_move_command(self, angle):
        # Calculate current angle difference
        angle_diff = radians(angle) - self.yaw
        angle_diff = (angle_diff + pi) % (2 * pi) - pi  # Normalize angle between -pi and pi

        # PID control for angle difference
        error_angle = angle_diff
        self.integral_angle += error_angle * self.dt
        derivative_angle = (error_angle - self.prev_error_angle) / self.dt

        # PID control law for angle
        correction_angle = (self.kp_angle * error_angle) + (self.ki_angle * self.integral_angle) + (self.kd_angle * derivative_angle)

        # Update previous error for angle control
        self.prev_error_angle = error_angle

        # Apply a saturation limit to the correction for angular velocity
        correction_angle = max(min(correction_angle, self.max_angular_vel), -self.max_angular_vel)

        # Create a Twist message to move the robot
        move_cmd = Twist()
        move_cmd.linear.x = 0.5  # Set linear velocity to 0.5 m/s
        move_cmd.angular.z = correction_angle  # Use PID output for angular velocity

        return move_cmd

if __name__ == '__main__':
    try:
        follower = GPSFollower()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
