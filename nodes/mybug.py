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
    (random.randrange(0,15),random.randrange(0,15)),
    (random.randrange(0,15),random.randrange(0,15)),
    (random.randrange(0,15),random.randrange(0,15)),
    (random.randrange(0,15),random.randrange(0,15)),
]

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

def closest_charger(rechargers, x, y):
    distance = map(lambda station: distance(p,(x,y)),rechargers)
    return filter(lambda closest: closest[0] == min(distance), zip(distance, rechargers))[0][1]

def distance(p1, p2):
    return math.sqrt((p1[0]-p2[0])**2 + (p1[1]-p2[1])**2)

class Bug:
    def __init__(self, gx, gy, battery):
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.goal = (tx,ty)
        self.temp = (None,None)
        self.start = (None,None)
        self.meters10=False
        self.greater10=True
        self.greater20=False
        self.notcontinueing=False
        self.speed = .5
        self.recharge = False
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
        #print "Going until destination or obstacle ", self.goal
        #while current_location.distance(tx, ty) > delta:
        (frontdist, _) = current_dists.get()
        if frontdist <= WALL_PADDING:
            self.statechange()
            self.follow_wall()

        if current_location.facing_point(*self.goal):
            self.go(STRAIGHT)
        elif current_location.faster_left(*self.goal):
            self.go(LEFT)
        else:
            self.go(RIGHT)
        rospy.sleep(.01)
        self.statechange()
        self.go_until_obstacle()

    def follow_wall(self):
        #print "Following wall"
        if current_dists.get()[0] <= WALL_PADDING:
            self.go(RIGHT)
            self.statechange()
            self.follow_wall()
            rospy.sleep(.01)
        if not self.should_leave_wall():
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
            self.statechange()
            self.follow_wall()
        else:
            self.statechange()
            self.go_until_obstacle()

    def statechange(self):
        if self.recharge and self.goal not in rechargers and distance(self.goal,current_location.current_location()[0:2]) > 10:
            self.gorecharge()
        #elif self.battery == 0:
            #print "Battery Reached Zero Robot Failed "
            #sys.exit(1)
        elif self.goal in rechargers and current_location.distance(*self.goal) <= (delta+.2):
            self.donerecharge();
        elif distance(self.goal,current_location.current_location()[0:2]) < 10 and not self.meters10 and not self.notcontinueing:
            ans = raw_input("Robot is less than 10 meters to goal. Should robot increase speed? (Y/N)")
            if ans == "y" or ans=="Y":
                self.speed = 1
            self.meters10=True
        elif distance(self.start,current_location.current_location()[0:2]) > 10 and self.greater10 and not self.notcontinueing:
            ans = raw_input("Robot is more than 10 meters from start. Should the robot continue? (Y/n)")
            if ans == "n" or ans == "N":
                self.speed = 0
            self.greater10 = False
        elif distance(self.start,current_location.current_location()[0:2]) > 20 and distance(self.goal,current_location.current_location()[0:2]) > 20 and not self.meters10 and not self.greater20 and not self.notcontinueing:
            ans = raw_input("Robot is more than 20 meters from goal and from start. press s to return to start or g to go to reach goal?")
            self.greater20=True
            if ans == "g" or ans == "G":
                print "Continuing to goal"
            else:
                self.meters10=True
                self.greater20=True
                self.gotostart()
        elif self.notcontinueing:
            self.gotostart()
        rospy.sleep(.1)

    def should_leave_wall(self):
        print "You dolt! You need to subclass bug to know how to leave the wall"
        sys.exit(1)

    def step(self):
        self.statechange()
        self.go_until_obstacle()
        rospy.sleep(.1)

    def battery_callback(self):
        self.battery -= 1;
        if self.battery < 30:
            self.recharge=True
  
    def timelimit_callback(self):
        self.continueing=True
        print "It has been 2 minutes. Going back to start"

    def recharged(self,point):
        self.battery=100;
        print "Recharge Battery at Station", point
   
    def printrecharge(self,point):
        print "Battery is bellow 30 to Recharge Station: ", point

    def printgostart(self,point):
        print "Failed the Mission Going to Start: ", point

    def gorecharge(self):
        self.temp=self.goal
        self.goal=closest_charger(rechargers,*current_location.current_location()[0:2])
        printrecharge(point)

    def donerecharge(self):
        recharged(self.goal)
        self.goal=self.temp

    def gotostart(self):
        self.goal=self.start
        printgostart(self.start)

    def should_leave_wall(self):
        (x, y, t) = current_location.current_location()
        g = current_location.global_to_local(necessary_heading(x, y, *self.goal))
        at = current_dists.at(g)
        (_, left) = current_dists.get()
        return at > 10# and left > WALL_PADDING

def near(cx, cy, x, y):
    nearx = x - .3 <= cx <= x + .3
    neary = y - .3 <= cy <= y + .3
    return nearx and neary

if __name__ == "__main__":
    if len(sys.argv) < 3:
        print "Usage: rosrun bugs mybug.py X Y"
        sys.exit(1)

    (tx, ty) = map(float, sys.argv[1:3])
    print "Setting target:", (tx, ty)
    bug = Bug(tx, ty,100)
    init_listener()
    print "Calibrating sensors..."
    # This actually just lets the sensor readings propagate into the system
    rospy.sleep(1)
    print "Calibrated"
    (sx, sy, _) = current_location.current_location()
    bug.start = (sx, sy)
    rospy.Timer(rospy.Duration(10), lambda _: bug.battery_callback)
    rospy.Timer(rospy.Duration(120), lambda _: bug.timelimit_callback)
    while current_location.distance(*bug.goal) > delta:
        bug.step()
