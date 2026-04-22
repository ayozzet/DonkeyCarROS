#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist

class Movement:

    def __init__(self):
        self.pub_cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.move_msg = Twist()
        self.rate = rospy.Rate(10)

    def publish_once_in_cmd_vel(self):
        while self.pub_cmd_vel.get_num_connections() == 0 and not rospy.is_shutdown():
            rospy.logwarn("Waiting for subscriber to /cmd_vel...")
            self.rate.sleep()

        self.pub_cmd_vel.publish(self.move_msg)

    def stop_car(self):
        self.move_msg.linear.x = 0.0
        self.move_msg.angular.z = 0.0
        self.publish_once_in_cmd_vel()

    def car_fwd(self):
        self.move_msg.linear.x = 0.2
        self.move_msg.angular.z = 0.0
        self.publish_once_in_cmd_vel()

    def car_rvs(self):
        self.move_msg.linear.x = -0.2
        self.move_msg.angular.z = 0.0
        self.publish_once_in_cmd_vel()

    def car_left(self):
        self.move_msg.linear.x = 0.0
        self.move_msg.angular.z = -1.0
        self.publish_once_in_cmd_vel()

    def car_right(self):
        self.move_msg.linear.x = 0.0
        self.move_msg.angular.z = 1.0
        self.publish_once_in_cmd_vel()

    def car_fwd_left(self):
        self.move_msg.linear.x = 0.2
        self.move_msg.angular.z = -0.2
        self.publish_once_in_cmd_vel()

    def car_fwd_right(self):
        self.move_msg.linear.x = 0.2
        self.move_msg.angular.z = 0.2
        self.publish_once_in_cmd_vel()


def main():
    rospy.init_node('movement_node', anonymous=True)
    movement = Movement()

    rospy.sleep(1)

    # Example test sequence
    movement.car_fwd()
    rospy.sleep(2)

    movement.car_left()
    rospy.sleep(2)

    movement.stop_car()


if __name__ == "__main__":
    main()