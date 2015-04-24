#!/usr/bin/env python

#Help from Ethan Miller

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

def distance(p1, p2):
    return (p1[0]-p2[0])**2 + (p1[1]-p2[1])**2

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

class Bug:
    def __init__(self, gx, gy, sx ,sy):
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.goal = (gx,gy)
        self.temp = (NONE,NONE)
        self.start = (sx,sy)
        self.battery = 100
        self.speed = .5
        self.meters10=False
        self.meters20=False
        self.greater20=False
        self.rechargers = [
            [random.randrange(0,20),random.randrange(0,20)],
            [random.randrange(0,20),random.randrange(0,20)],
            [random.randrange(0,20),random.randrange(0,20)],
            [random.randrange(0,20),random.randrange(0,20)],
        ]
        self.state = "GO_UNTIL_OBSTACLE"
        self.states = {
            'GO_UNTIL_OBSTACLE': lambda x: x.go_until_obstacle(),
            'FOLLOW_WALL': lambda x: x.follow_wall(),
            'RECHARGE': lambda x: x.recharging(),
            'RECHARGED': lambda x: x.recharged(),
            'TOSTART': lambda x: x.back_to_start(),
            'DEATH': lambda x: x.death(),
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
        dir_to_go = current_location.global_to_local(necessary_heading(x, y,*self.goal))
        at = current_dists.at(dir_to_go)
        (_, left) = current_dists.get()
        return at > 10

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
        self.temp = self.goal
        self.goal = self.closest_charger(self.rechargers,*current_location.current_location()[0:2])
        print "Going to Charging Station at ", self.goal
        return "GO_UNTIL_OBSTACLE"

    def recharged(self):
        self.goal=self.temp
        self.battery=100
        print "Done Charging going to Goal", self.goal
        return "GO_UNTIL_OBSTACLE"

    def back_to_start(self):
        self.goal=self.start
        return "GO_UNTIL_OBSTACLE"

    def death(self):
        self.speed=0
        self.goal=current_location.current_location()[0:2]

    def statechange(self):
        if self.battery < 30  and self.goal not in self.rechargers and distance(self.goal,*current_location.current_location()[0:2]) < 10:
            return "RECHARGE"
        if self.battery == 0:
            print "Battery Reached Zero Robot Failed"
            return "DEATH"
        if self.goal in self.rechargers and current_location.current_location(*self.goal) <= (delta+.2):
            return "RECHARGED"
        if distance(self.goal,*current_location.current_location()[0:2]) < 10 and not self.meters10:
            ans = raw_input("Robot is less than 10 meters to goal. Should robot increase speed? (Y/N)")
            if ans == "y" or ans=="Y":
                self.speed = 1
            self.meters10=True
        if distance(self.start,*current_location.current_location()[0:2]) > 20 and distance(self.goal,*current_location.current_location()[0:2]) > 20 and not self.meters10 and not self.meters20 and not self.greater20:
            ans = raw_input("Robot is more than 20 meters from goal and from start. press s to return to start or g to go to reach goal?")
            greater20=True
            if ans == "g" or ans == "G":
                print "Continuing to goal"
            else:
                print "Returning to Start"
                return "TOSTART"
        return "GO_UNTIL_OBSTACLE"
        rospy.sleep(.1)

    def closest_charger(rechargers, x, y):
    distance = map(lambda station: (station[0]-x)**2 + (station[1]-y)**2,rechargers)
    return filter(lambda closest: closest[0] == min(distance), zip(distance, rechargers))[0][1]

    def battery_callback(self):
    self.battery -= 1;
    print "Battery Left " + str(self.battery)

    def timelimit_callback(self):
        print "It has been 2 minutes. Going back to start"
        return "TOSTART"

    def step(self):
        self.state=self.statechange()
        #self.state = self.states[self.state](self) # did I stutter?
        rospy.sleep(.1)

if __name__ == "__main__":
    if len(sys.argv) < 3:
        print "Usage: rosrun bugs bug.py X Y"
        sys.exit(1)

    (gx, gy) = map(float, sys.argv[1:3])
    print "Setting target:", (gx, gy)
    init_listener()
    (sx, sy, _) = current_location.current_location()
    bug = Bug(gx, gy, sx, sy)
    print "Calibrating sensors..."
    # This actually just lets the sensor readings propagate into the system
    print "Calibrated"
    rospy.Timer(rospy.Duration(10), lambda _: bug.battery_callback())
    rospy.Timer(rospy.Duration(120), lambda _: bug.timelimit_callback())

    while current_location.distance(*bug.goal) > delta:
        bug.step()
