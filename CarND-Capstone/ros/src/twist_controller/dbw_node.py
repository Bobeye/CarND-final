#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool
from dbw_mkz_msgs.msg import ThrottleCmd, SteeringCmd, BrakeCmd, SteeringReport
from geometry_msgs.msg import TwistStamped, PoseStamped
from styx_msgs.msg import Lane, Waypoint
import math
import numpy as np
import tf

from twist_controller import Controller
from pid import PID 

'''
You can build this node only after you have built (or partially built) the `waypoint_updater` node.

You will subscribe to `/twist_cmd` message which provides the proposed linear and angular velocities.
You can subscribe to any other message that you find important or refer to the document for list
of messages subscribed to by the reference implementation of this node.

One thing to keep in mind while building this node and the `twist_controller` class is the status
of `dbw_enabled`. While in the simulator, its enabled all the time, in the real car, that will
not be the case. This may cause your PID controller to accumulate error because the car could
temporarily be driven by a human instead of your controller.

We have provided two launch files with this node. Vehicle specific values (like vehicle_mass,
wheel_base) etc should not be altered in these files.

We have also provided some reference implementations for PID controller and other utility classes.
You are free to use them or build your own.

Once you have the proposed throttle, brake, and steer values, publish it on the various publishers
that we have created in the `__init__` function.

'''

class DBWNode(object):
    def __init__(self):
        rospy.init_node('dbw_node')

        vehicle_mass = rospy.get_param('~vehicle_mass', 1736.35)
        fuel_capacity = rospy.get_param('~fuel_capacity', 13.5)
        brake_deadband = rospy.get_param('~brake_deadband', .1)
        decel_limit = rospy.get_param('~decel_limit', -5)
        accel_limit = rospy.get_param('~accel_limit', 1.)
        wheel_radius = rospy.get_param('~wheel_radius', 0.2413)
        wheel_base = rospy.get_param('~wheel_base', 2.8498)
        steer_ratio = rospy.get_param('~steer_ratio', 14.8)
        max_lat_accel = rospy.get_param('~max_lat_accel', 3.)
        max_steer_angle = rospy.get_param('~max_steer_angle', 8.)

        self.steer_pub = rospy.Publisher('/vehicle/steering_cmd',
                                         SteeringCmd, queue_size=1)
        self.throttle_pub = rospy.Publisher('/vehicle/throttle_cmd',
                                            ThrottleCmd, queue_size=1)
        self.brake_pub = rospy.Publisher('/vehicle/brake_cmd',
                                         BrakeCmd, queue_size=1)

        min_speed = 0.


        # TODO: Create `TwistController` object
        # self.controller = TwistController(<Arguments you wish to provide>)
        self.Tcontroller = Controller(wheel_base = wheel_base, 
                                      steer_ratio = steer_ratio, 
                                      min_speed = min_speed, 
                                      max_lat_accel = max_lat_accel, 
                                      max_steer_angle = max_steer_angle)


        self.current_velocity = 0.
        self.velocity_ref = 10.
        self.final_waypoints = None
        self.current_pose = None
        self.cte = 0
        self.vel_control=0.
        self.throttle = 0
        self.brake = 0
        self.steering = 0
        self.time_current = 0
        self.time_previous = 0
        self.time_delta = 0

        self.velocity_pid = PID(kp=1., ki=0.01, kd=0.)
        self.steering_pid = PID(kp=10., ki=0.0, kd=0., mn=-max_steer_angle, mx=max_steer_angle)

        # TODO: Subscribe to all the topics you need to
        rospy.Subscriber('/vehicle/dbw_enabled', Bool, self.dbw_cb)
        rospy.Subscriber('/current_velocity', TwistStamped, self.current_vel_cb, queue_size=1)
        rospy.Subscriber('/current_pose', PoseStamped, self.current_pose_cb, queue_size=1)
        rospy.Subscriber('/final_waypoints', Lane, self.final_waypoints_cb)


        self.loop()



    def loop(self):
        rate = rospy.Rate(50) # 50Hz
        while not rospy.is_shutdown():
            # TODO: Get predicted throttle, brake, and steering using `twist_controller`
            # You should only publish the control commands if dbw is enabled
            # throttle, brake, steering = self.controller.control(<proposed linear velocity>,
            #                                                     <proposed angular velocity>,
            #                                                     <current linear velocity>,
            #                                                     <dbw status>,
            #                                                     <any other argument you need>)
            # if <dbw is enabled>:
            #   self.publish(throttle, brake, steer)
            
            if self.final_waypoints is not None:
                self.time_current = rospy.get_rostime().secs + rospy.get_rostime().nsecs/1000000000.
                self.time_delta = self.time_current-self.time_previous
                self.time_previous = self.time_current
                self.cte = self.cross_track_error(self.final_waypoints, self.current_pose)
                self.steering = self.steering_pid.step(self.cte, self.time_delta)
                self.vel_control = self.velocity_pid.step(-self.current_velocity+self.velocity_ref, self.time_delta)
            # if self.vel_control>0:         
            #     self.throttle = self.vel_control
            #     self.brake = 0
            # else:
            #     self.throttle = 0
            #     self.brake = abs(self.vel_control)*20000
                if self.current_velocity < 10.:
                    self.throttle = 2.
                    self.brake = 0.
                else:
                    self.throttle = 0.
                    self.brake = 0.

            
            # rospy.logwarn(self.steering)
            # rospy.logwarn(self.cte)
            # if <dbw is enabled>:
            self.publish(self.throttle, self.brake, self.steering)
            rate.sleep()

    def publish(self, throttle, brake, steer):
        tcmd = ThrottleCmd()
        tcmd.enable = True
        tcmd.pedal_cmd_type = ThrottleCmd.CMD_PERCENT
        tcmd.pedal_cmd = throttle
        self.throttle_pub.publish(tcmd)

        scmd = SteeringCmd()
        scmd.enable = True
        scmd.steering_wheel_angle_cmd = steer
        self.steer_pub.publish(scmd)

        bcmd = BrakeCmd()
        bcmd.enable = True
        bcmd.pedal_cmd_type = BrakeCmd.CMD_TORQUE
        bcmd.pedal_cmd = brake
        self.brake_pub.publish(bcmd)

    def cross_track_error(self, waypoints, pose):
        # get future waypoints
        ptsX = [point.pose.pose.position.x for point in waypoints]
        ptsY = [point.pose.pose.position.y for point in waypoints]
        # get vehicle yaw angle, Quaternion to Euler
        quaternion = (pose.orientation.x,
                      pose.orientation.y,
                      pose.orientation.z,
                      pose.orientation.w)
        yawV = tf.transformations.euler_from_quaternion(quaternion)[2]

        # transfer to vehicle coordinates
        px = pose.position.x
        py = pose.position.y
        ptsXV = []
        ptsYV = []
        for i in range(len(ptsX)):
            temp_x = (ptsY[i]-py)*np.sin(yawV) - (px-ptsX[i])*np.cos(yawV)
            temp_y = (ptsY[i]-py)*np.cos(yawV) + (px-ptsX[i])*np.sin(yawV)
            ptsXV += [temp_x]
            ptsYV += [temp_y]
        coeffs = np.polyfit(ptsXV, ptsYV, 5)[::-1]
        cte = 0
        for i in range(len(coeffs)):
            cte += coeffs[i]*(self.current_velocity*0.44704*self.time_delta**i)
        # cte = np.polyval(coeffs, self.current_velocity*0.44704*self.time_delta)
        # rospy.logwarn(cte)
        return cte

    def dbw_cb(self, msg):
        if msg.data:
            self.dbw_enabled = True
        else:
            self.dbw_enabled = False

    def current_vel_cb(self, msg):
        self.current_velocity = msg.twist.linear.x

    def current_pose_cb(self, msg):
        self.current_pose = msg.pose 

    def final_waypoints_cb(self, msg):
        self.final_waypoints = msg.waypoints 

if __name__ == '__main__':
    DBWNode()