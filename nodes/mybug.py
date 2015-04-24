#!/usr/bin/env python

import math
import sys
import random
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

rechargers = [
    (random.randrange(0,20),random.randrange(0,20)),
    (random.randrange(0,20),random.randrange(0,20)),
    (random.randrange(0,20),random.randrange(0,20)),
    (random.randrange(0,20),random.randrange(0,20)),
]

def battery_callback(event):
    bug.battery -= 1;
    print "Battery Left " + str(bug.battery)
    if bug.battery < 30:
        bug.recharge=True
  
def timelimit_callback(event):
    bug.continueing=True
    print "It has been 2 minutes. Going back to start"

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
    t = transform.euler_from_quaternion(q)[2]
    current_location.update_location(p.x, p.y, t)


def sensor_callback(data):
    current_dists.update(data)

def recharged(point):
    Bug.battery=100;
    print "Recharge Battery at Station", point
   
def printrecharge(point):
    print "Battery is bellow 30 to Recharge Station: ", point

def printgostart(point):
    print "Failed the Mission Going to Start: ", point

def gorecharge(point):
    bug.temp=bug,goal
    bug.goal=point
    printrecharge(point)

def donerecharge():
    bug,goal=bug.temp
    recharged(bug.temp)

def gotostart():
    bug.goal=bug.start
    printgostart(bug.start)

def closest_charger(rechargers, x, y):
    distance = map(lambda station: (station[0]-x)**2 + (station[1]-y)**2,rechargers)
    return filter(lambda closest: closest[0] == min(distance), zip(distance, rechargers))[0][1]

def distance(p1, p2):
    return (p1[0]-p2[0])**2 + (p1[1]-p2[1])**2

class Bug:
    def __init__(self, gx, gy, battery):
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.goal = (tx,ty)
        self.temp = (None,None)
        self.start = (None,None)
        self.meters10=False
        self.meters20=False
        self.greater20=False
        self.continueing=False
        self.speed = .5
        self.battery = battery

    def go(self, direction):
        cmd = Twist()
        if direction == STRAIGHT:
            cmd.linear.x = self.speed
        elif direction == LEFT:
            cmd.angular.z = 0.25
        elif direction == RIGHT:
            cmd.angular.z = -0.25
        elif direction == MSG_STOP:
            pass
        self.pub.publish(cmd)

    def go_until_obstacle(self):
        print "Going until destination or obstacle" + str(tx) + " " + str(ty)
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
        return at > 10

def near(cx, cy, x, y):
    nearx = x - .3 <= cx <= x + .3
    neary = y - .3 <= cy <= y + .3
    return nearx and neary

def bug_algorithm(bug):
    init_listener()
    print "Calibrating sensors..."
    # This actually just lets the sensor readings propagate into the system
    rospy.sleep(1)
    bug.start = current_location.current_location()[0:2]
    print "Calibrated: Start Position " , bug.start
    rospy.Timer(rospy.Duration(10), battery_callback)
    rospy.Timer(rospy.Duration(120), timelimit_callback)
    while current_location.distance(tx, ty) > delta:
        if bug.battery < 30 and bug.goal not in rechargers and distance(bug.goal,current_location.current_location()[0:2]) < 10:
            recharge(closest_charger(rechargers,*current_location.current_location()[0:2]))
        if bug.battery == 0:
            print "Battery Reached Zero Robot Failed"
            sys.exit(1)
        if bug.goal in rechargers and current_location.current_location(*bug.goal) <= (delta+.2):
            recharged();
        if distance(bug.goal,current_location.current_location()[0:2]) < 10 and not bug.meters10:
            ans = raw_input("Robot is less than 10 meters to goal. Should robot increase speed? (Y/N)")
            if ans == "y" or ans=="Y":
                bug.speed = 1
            bug.meters10=True
        if distance(bug.start,current_location.current_location()[0:2]) > 20 and distance(bug.goal,current_location.current_location()[0:2]) > 20 and not bug.meters10 and not bug.meters20 and not bug.greater20:
            ans = raw_input("Robot is more than 20 meters from goal and from start. press s to return to start or g to go to reach goal?")
            bug.greater20=True
            if ans == "g" or ans == "G":
                print "Continuing to goal"
            else:
                gotostart()
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
