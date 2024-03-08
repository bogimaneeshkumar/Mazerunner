#!/usr/bin/env python
import math
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion
from std_msgs.msg import Float64


class AGV():
    def __init__(self):
        rospy.init_node("gmapping", anonymous=False)
        rospy.Subscriber("/scan", LaserScan, self.laser_callback)
        rospy.Subscriber("/odom", Odometry, self.odom_callback)

        self.left_wheel_pub = rospy.Publisher(
            '/mobot/Left_wheel_velocity/command', Float64, queue_size=10)
        self.right_wheel_pub = rospy.Publisher(
            '/mobot/Right_wheel_velocity/command', Float64, queue_size=10)

        self.laser = LaserScan()
        self.odom_data = Odometry()

        self.regions = {
            'right': 0,
            'fright': 0,
            'front': 0,
            'fleft': 0,
            'bleft': 0,
            'turn': 0,
            'margin' : 0
        }

        self.max_ = 5
        self.yaw = 0.0

        self.count = 0
        self.count2 = 0

        self.case = ""

        self.linear_velocity = 0.0
        self.angular_velocity = 0.0

        self.max_radius = 0.35

        self.flag = 0
        self.flag_2 = 0

    def laser_callback(self, laser_msg):
        self.laser = laser_msg
        # dividing laser feedback regions into 5 parts
        self.regions = {
            'bright': min(min(self.laser.ranges[0:143]), self.max_),
            'fright': min(min(self.laser.ranges[144:287]), self.max_),
            'front': min(min(self.laser.ranges[288:431]), self.max_),
            'fleft': min(min(self.laser.ranges[432:576]), self.max_),
            'bleft': min(min(self.laser.ranges[577:719]), self.max_),
            'turn': min(min(self.laser.ranges[670:719]), self.max_),
            'margin' : min(min(self.laser.ranges[355:365]), 10)
        }

    def odom_callback(self, odom):
        self.odom_data = odom
        self.x_now = self.odom_data.pose.pose.position.x
        self.y_now = self.odom_data.pose.pose.position.y
        x = self.odom_data.pose.pose.orientation.x
        y = self.odom_data.pose.pose.orientation.y
        z = self.odom_data.pose.pose.orientation.z
        w = self.odom_data.pose.pose.orientation.w
        euler = euler_from_quaternion([x, y, z, w])
        self.yaw = euler[2]  # Yaw angle for the robot
        self.yaw = self.angle_change(self.yaw)
        # The current coordinate system is changed to cartesian coordinate system
        # and by doing so angles are changed to [0,2*pi]

    def angle_change(self, phi):  # Function used to change angle ranges in the coordinate system
        if phi < 0:
            phi = 2 * math.pi + phi
        return phi

    def laser_feedback(self):

        if self.regions['bleft'] < 1.8:
            self.case = "fwd"
            if self.regions['bleft'] < 1.2:
                self.case = "rotate"
            if self.regions['front'] < 1:
                self.case = "rotate"
        else:
            if self.regions['turn'] > 1.8:
                self.case = "left"
            else:
                self.case = "fwd"

    def move_forward(self):    # Function used to move forward
        self.left_wheel_pub.publish(5)
        self.right_wheel_pub.publish(5)

    def stop(self):           # Function used to stop the robot
        self.left_wheel_pub.publish(0)
        self.right_wheel_pub.publish(0)

    def rotate_to_align(self):  # Function used to allign the robot along the trough
        while True:
            if self.regions['bleft'] < 0.5 and self.regions['fleft'] > 0.5:
                break
            self.linear_velocity = 2
            self.angular_velocity = 3.5
            self.left_wheel_pub.publish(
                self.linear_velocity + self.angular_velocity)
            self.right_wheel_pub.publish(self.linear_velocity)

    def rotate_left(self):  # Function used to move left
        self.linear_velocity = 2
        self.angular_velocity = 3.7
        self.left_wheel_pub.publish(self.linear_velocity)
        self.right_wheel_pub.publish(
            self.linear_velocity + self.angular_velocity)
    
    def rotate(self):
        self.left_wheel_pub.publish(2)
        self.right_wheel_pub.publish(-2)

    def rotate_l(self):
        self.left_wheel_pub.publish(2)
        self.right_wheel_pub.publish(4)

    def rot(self):
        self.left_wheel_pub.publish(2.5)
        self.right_wheel_pub.publish(0.5)

    def cntrl2(self):
        while not rospy.is_shutdown():
            self.laser_feedback()

            if self.regions['front'] < 1 or self.regions['fright'] < 0.8:
                self.rotate()
                print("rotating")
            else:
                print(self.case)
                if self.case == "fwd":
                    self.move_forward()
                    self.flag = 0
                if self.case == "left":
                    if self.regions['margin'] > 3.5 :
                        self.rotate_l
                    else :
                        if self.regions['front']>2 and self.flag == 0:
                            self.rot()
                        else : 
                            self.rotate_left()
                            self.flag = 1
                        

                    self.rot()
                    self.rotate_left()
                if self.case == "rotate":
                    self.rotate()


if __name__ == '__main__':
    try:
        rospy.sleep(10)
        agv = AGV()
        agv.cntrl2()
    except rospy.ROSInterruptException:
        pass
