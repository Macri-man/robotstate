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

time = 0
state = 0
sx = 0
sy = 0

def battery_callback(event):
    print 'time for battery to drain' + str(event.current_real)

def timelimit_callback(event):
    print 'time to end this' + str(event.current_real)

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
    if state == None:
        pass
    elif state = 2 and current_location.distance(sx,sy) > 20:
        print "Should Robot continue ot goal?(G) or return to start (S)"
        name = raw_input("Should Robot continue ot goal?(G) or return to start (S)")
    elif state = 1
        print "Should I go further? (Y/N) "
        name = raw_input("Should I go further? (Y/N) ")
    elif state = 0
        print "Should I go faster? (Y/N) "
        name = raw_input("Should I go faster? (Y/N) ")


def sensor_callback(data):
    current_dists.update(data)

class Bug:
    def __init__(self, tx, ty, battery):
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.tx = tx
        self.ty = ty
        self.battery = battery

    def go(self, direction):
        cmd = Twist()
        if direction == STRAIGHT:
            cmd.linear.x = 1
        elif direction == LEFT:
            cmd.angular.z = 0.25
        elif direction == RIGHT:
            cmd.angular.z = -0.25
        elif direction == MSG_STOP:
            pass
        self.pub.publish(cmd)

    def go_until_obstacle(self):
        print "Going until destination or obstacle"
        while current_location.distance(tx, ty) > delta:
            (frontdist, _) = current_dists.get()
            if frontdist <= WALL_PADDING:
                return True

            if current_location.facing_point(tx, ty):
                self.go(STRAIGHT)
            elif current_location.faster_left(tx, ty):
                self.go(LEFT)
            else:
                self.go(RIGHT)
            rospy.sleep(.01)
        return False

    def follow_wall(self):
        print "Following wall"
        while current_dists.get()[0] <= WALL_PADDING:
            self.go(RIGHT)
            rospy.sleep(.01)
        while not self.should_leave_wall():
            (front, left) = current_dists.get()
            if front <= WALL_PADDING:
                self.go(RIGHT)
            elif WALL_PADDING - .1 <= left <= WALL_PADDING + .1:
                self.go(STRAIGHT)
            elif left > WALL_PADDING + .1:
                self.go(LEFT)
            else:
                self.go(RIGHT)
            rospy.sleep(.01)

    def should_leave_wall(self):
        print "You dolt! You need to subclass bug to know how to leave the wall"
        sys.exit(1)

class Bug0(Bug):
    def should_leave_wall(self):
        (x, y, t) = current_location.current_location()
        dir_to_go = current_location.global_to_local(necessary_heading(x, y, tx, ty))
        at = current_dists.at(dir_to_go)
        if at > 10 or self.battery < 30:
            print "Leaving wall"
            return True
        return False

def near(cx, cy, x, y):
    nearx = x - .3 <= cx <= x + .3
    neary = y - .3 <= cy <= y + .3
    return nearx and neary

def bug_algorithm(bug):
    init_listener()
    print "Calibrating sensors..."
    # This actually just lets the sensor readings propagate into the system
    rospy.sleep(1)
    sx = current_location.x
    sy = current_location.y
    print "Calibrated"
    if current_location.distance(tx, ty) > 20:
        state = 2
    elif 20 > current_location.distance(tx, ty) > 10:
        state = 1
    elif 20 > current_location.distance(tx, ty) > 10:
        state = 0

    while current_location.distance(tx, ty) > delta:
        rospy.Timer(rospy.Duration(10), battery_callback)
        rospy.Timer(rospy.Duration(120), timelimit_callback)
        hit_wall = bug.go_until_obstacle()
        if hit_wall:
            bug.follow_wall()
    print "Arrived at", (tx, ty)

# Parse arguments

if len(sys.argv) < 3:
    print "Usage: rosrun bugs bug.py X Y"
    sys.exit(1)
(tx, ty) = map(float, sys.argv[1:3])



print "Setting target:", (tx, ty)
bug = Bug0(tx, ty, 100)

bug_algorithm(bug)