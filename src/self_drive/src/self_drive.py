#!/home/pi/.pyenv/versions/rospy3/bin/python

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class SelfDrive:
    def __init__(self, publisher):
        self.publisher = publisher
        self.count = 30
        self.turtle_vel = Twist()


    def lds_callback(self, scan):

        print("front : ", scan.ranges[0])
        turtle_vel = Twist()

        if scan.ranges[0] < 0.35:
            self.stop()
        else:
            self.go_straight()

        self.publisher.publish(turtle_vel)

    def go_straight(self):
        self.turtle_vel.linear.x = 0.15
        self.turtle_vel.angular.z = 0.0
        self.publisher.publish(self.turtle_vel)

    def stop(self):
        self.turtle_vel.linear.x = 0.0
        self.turtle_vel.angular.z = 0.0
        self.publisher.publish(self.turtle_vel)


def main():
    rospy.init_node('self_drive')
    publisher = rospy.Publisher('cmd_vel', Twist, queue_size=1)
    driver = SelfDrive(publisher)
    subscriber = rospy.Subscriber('scan', LaserScan,
                                  lambda scan: driver.lds_callback(scan))
    rospy.spin()


if __name__ == "__main__":
    main()

