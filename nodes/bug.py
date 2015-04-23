#!/usr/bin/env python

import math
import sys
import roslib; roslib.load_manifest('bugs')
import rospy
import tf.transformations as transform

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

from location import Location, necessary_heading
from dist import Dist

current_location = Location()
current_dists = Dist()

delta = .1
WALL_PADDING = .5

STRAIGHT = 0
LEFT = 1
RIGHT = 2
MSG_STOP = 3

def init_listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber('base_pose_ground_truth', Odometry, location_callback)
    rospy.Subscriber('base_scan', LaserScan, sensor_callback)

def location_callback(data):
    p = data.pose.pose.position
    q = (
            data.pose.pose.orientation.x,
            data.pose.pose.orientation.y,
            data.pose.pose.orientation.z,
            data.pose.pose.orientation.w)
    t = transform.euler_from_quaternion(q)[2] # in [-pi, pi]
    current_location.update_location(p.x, p.y, t)

def sensor_callback(data):
    current_dists.update(data)

class Bug:
    def __init__(self, gx, gy, sx ,sy):
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.goal = (gx,gy)
        self.temp = (NONE,NONE)
        self.start = (sx,sy)
        self.battery = 100
        self.speed = .5
        self.state = "GO_UNTIL_OBSTACLE"
        self.states = {
            'GO_UNTIL_OBSTACLE': lambda x: x.go_until_obstacle(),
            'FOLLOW_WALL': lambda x: x.follow_wall(),
            'RECHARGE': lambda x: x.recharging(),
            'DEATH': lambda x: x.death(),
            'GO_TO_START': lambda x: x.back_to_start(),
        }

    def go_until_obstacle(self):
        (front, _) = current_dists.get()
        if front <= WALL_PADDING:
            return "FOLLOW_WALL"
        if current_location.facing_point(self.goal):
            self.go(STRAIGHT, self.speed)
        elif current_location.faster_left(self.goal):
            self.go(LEFT, self.speed)
        else:
            self.go(RIGHT, self.speed)
        return "GO_UNTIL_OBSTACLE"

    def follow_wall(self):
        if current_dists.get()[0] <= WALL_PADDING:
            self.go(RIGHT, self.speed)
            return "FOLLOW_WALL"
        if not self.should_leave_wall():
            (front, left) = current_dists.get()
            if front <= WALL_PADDING:
                self.go(RIGHT, self.speed)
            elif WALL_PADDING - .1 <= left <= WALL_PADDING + .1:
                self.go(STRAIGHT, self.speed)
            elif left > WALL_PADDING + .1:
                self.go(LEFT, self.speed)
            else:
                self.go(RIGHT, self.speed)
            return "FOLLOW_WALL"
        else:
            return "GO_UNTIL_OBSTACLE"

    def should_leave_wall(self):
        (x, y, t) = current_location.current_location()
        dir_to_go = current_location.global_to_local(necessary_heading(x, y,self.goal.x,self.goal.y))
        at = current_dists.at(dir_to_go)
        (_, left) = current_dists.get()
        if at > 10 and left > 10:
            print "Leaving wall"
            return True
        return False

    def go(self, direction, speed):
        cmd = Twist()
        if direction == STRAIGHT:
            cmd.linear.x = speed
        elif direction == LEFT:
            cmd.angular.z = 0.25
        elif direction == RIGHT:
            cmd.angular.z = -0.25
        elif direction == MSG_STOP:
            pass
        self.pub.publish(cmd)

    def recharging(self):
        self.temp=self.goal

    def back_to_start(self):
        self.goal=self.start
        return "GO_UNTIL_OBSTACLE"

    def death(self):
        self.goal=self.start
        return "GO_UNTIL_OBSTACLE"

    def battery_callback(self):
    self.battery = self.battery-1;
    print "Battery Left" + str(self.battery)
    if self.battery < 30:
        return "RECHARGE"
    elif self.battery == 30:
        self.speed=0
        print "Battery Reached Zero Robot Failed"
        return "DEATH"

    def timelimit_callback(self):
        print "It has been 2 minutes. Going back to start"
        return "GO_TO_START"


    def step(self):
        self.state = self.states[self.state](self) # did I stutter?
        rospy.sleep(.1)

if __name__ == "__main__":
    if len(sys.argv) < 3:
        print "Usage: rosrun bugs bug.py X Y"
        sys.exit(1)

    (gx, gy) = map(float, sys.argv[1:3])
    print "Setting target:", (gx, gy)
    bug = Bug(gx, gy)
    init_listener()
    print "Calibrating sensors..."
    # This actually just lets the sensor readings propagate into the system
    (ix, iy, _) = current_location.current_location()
    bug.initial = (ix, iy)
    print "Calibrated"
    rospy.Timer(rospy.Duration(10), lambda _: bug.battery_callback())
    rospy.Timer(rospy.Duration(120), lambda _: bug.timelimit_callback())

    while current_location.distance(self.goal) > delta:
        bug.step()
