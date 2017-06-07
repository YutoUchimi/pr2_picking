#!/usr/bin/env python

PKG = "pr2_mechanism_controllers"

import roslib; roslib.load_manifest(PKG)
import sys
import os
import string
import rospy
from std_msgs import *
from pr2_msgs.msg import LaserTrajCmd
from pr2_msgs.srv import *
from time import sleep


def print_usage(exit_code = 0):
    print '''Usage:
    send_periodic_cmd.py [controller]
'''
    sys.exit(exit_code)


if __name__ == '__main__':
    if len(sys.argv) < 2:
        print_usage()

    cmd = LaserTrajCmd()
    controller = sys.argv[1]
    cmd.header = rospy.Header(None, None, None)
    cmd.profile = "blended_linear"

    cycle_time = rospy.get_param('~cycle_time', 5.0)
    min_tilt_angle = rospy.get_param('~min_tilt_angle', -0.7)
    max_tilt_angle = rospy.get_param('~max_tilt_angle', 1.0)

    cmd.position = [min_tilt_angle, max_tilt_angle, min_tilt_angle]
    cmd.time_from_start = [0.0,  cycle_time-1.0, cycle_time]
    cmd.time_from_start = [rospy.Duration.from_sec(x) for x in cmd.time_from_start]
    cmd.max_velocity = 10
    cmd.max_acceleration = 30

    print 'Sending Command to %s: ' % controller
    print '  Profile Type: %s' % cmd.profile
    print '  Pos: %s ' % ','.join(['%.3f' % x for x in cmd.position])
    print '  Time: %s' % ','.join(['%.3f' % x.to_sec() for x in cmd.time_from_start])
    print '  MaxRate: %f' % cmd.max_velocity
    print '  MaxAccel: %f' % cmd.max_acceleration

    rospy.wait_for_service(controller + '/set_traj_cmd')

    s = rospy.ServiceProxy(controller + '/set_traj_cmd', SetLaserTrajCmd)
    resp = s.call(SetLaserTrajCmdRequest(cmd))

    print 'Command sent!'
    print '  Resposne: %f' % resp.start_time.to_sec()
