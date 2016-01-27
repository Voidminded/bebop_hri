#!/usr/bin/env python

# Requires these topics (bebop atittude, vicon data (rpy should be REP 103), and cmd_vel_twist (requires post processing of cmd_vel) and also speed)

import csv
from collections import deque

import rospy
import message_filters
import tf
from bebop_msgs.msg import Ardrone3PilotingStateAttitudeChanged
from bebop_msgs.msg import Ardrone3PilotingStateSpeedChanged
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import TwistStamped

import math
import sys

# http://dsp.stackexchange.com/questions/9498/have-position-want-to-calculate-velocity-and-acceleration

import numpy as np
import numpy.linalg

def sg_filter(x, m, k=0):
    """
    x = Vector of sample times
    m = Order of the smoothing polynomial
    k = Which derivative
    """
    mid = len(x) / 2
    a = x - x[mid]
    expa = lambda x: map(lambda i: i**x, a)
    A = np.r_[map(expa, range(0,m+1))].transpose()
    Ai = np.linalg.pinv(A)

    return Ai[k]

def smooth(x, y, size=5, order=2, deriv=0):

    if deriv > order:
        raise Exception, "deriv must be <= order"

    n = len(x)
    m = size

    result = np.zeros(n)

    for i in xrange(m, n-m):
        start, end = i - m, i + m + 1
        f = sg_filter(x[start:end], order, deriv)
        result[i] = np.dot(f, y[start:end])

    if deriv > 1:
        result *= math.factorial(deriv)

    return result

smoothing_window_size = 15
smoothing_order = 5

time_l = list()
vic_x_w = list()
vic_y_w = list()
vic_z_w = list()
vic_roll_w = list()
vic_pitch_w = list()
vic_yaw_w = list()
vic_vx_w_smoothed = list()
vic_vy_w_smoothed = list()
vic_vz_w_smoothed = list()
vic_vx_b_smoothed = list()
vic_vy_b_smoothed = list()
vic_ax_w_smoothed = list()
vic_ay_w_smoothed = list()
vic_az_w_smoothed = list()
vic_ax_b_smoothed = list()
vic_ay_b_smoothed = list()
beb_vx_enu = list()
beb_vy_enu = list()
beb_vz_enu = list()
beb_vx_b = list()
beb_vy_b = list()
beb_roll_b = list()
beb_pitch_b = list()
beb_yaw_b = list()
cvel_rolln_b = list()
cvel_pitchn_b = list()
cvel_vzn_b = list()
cvel_vyawn_b = list()

def sync_callback(att, cmd_vel, trans, speed):
    rospy.loginfo("In Sync!")

    q = (trans.transform.rotation.x, trans.transform.rotation.y, trans.transform.rotation.z, trans.transform.rotation.w)
    [vic_r_b, vic_p_b, vic_y_b] = tf.transformations.euler_from_quaternion(q)

    # convert all poses to world coordinate system (vicon)
    # convert all angles to bebop local (REP 103)
    # cmd_vel.x changes the pitch to move bebop forward, same for .y

    time_l.append(att.header.stamp.to_sec())
    vic_x_w.append(trans.transform.translation.x)
    vic_y_w.append(trans.transform.translation.y)
    vic_z_w.append(trans.transform.translation.z)
    vic_roll_w.append(vic_r_b)
    vic_pitch_w.append(vic_p_b)
    vic_yaw_w.append(vic_y_b)
    # hypo: these are in ESD coordinates, let's make z upward and make it ENU
    # x: east, y: north, z: up
    # (original: x: east, y: south, z: down)
    beb_vx_enu.append(speed.speedX)
    beb_vy_enu.append(-speed.speedY)
    beb_vz_enu.append(-speed.speedZ)

    # phi: beb_yaw_b
    phi = -att.yaw
    vx = beb_vx_enu[-1]
    vy = beb_vy_enu[-1]
    beb_vx_b.append(vx * math.cos(phi) + vy * math.sin(phi))
    beb_vy_b.append(-vx * math.sin(phi) + vy * math.cos(phi))

    beb_roll_b.append(att.roll)
    beb_pitch_b.append(-att.pitch)
    beb_yaw_b.append(-att.yaw)
    cvel_rolln_b.append(cmd_vel.twist.linear.y)
    cvel_pitchn_b.append(cmd_vel.twist.linear.x)
    cvel_vzn_b.append(cmd_vel.twist.linear.z)
    cvel_vyawn_b.append(cmd_vel.twist.angular.z)

def main():
    rospy.init_node('bebop_iden', anonymous=True)

    # use queue_size about 20 times the update rate
    att_sub = message_filters.Subscriber('bebop/states/ARDrone3/PilotingState/AttitudeChanged', Ardrone3PilotingStateAttitudeChanged, queue_size=800)
    cmd_vel_sub = message_filters.Subscriber('bebop/cmd_vel_stamped', TwistStamped, queue_size = 2500)
    trans_sub = message_filters.Subscriber('vicon/bebop_blue_en/bebop_blue_en', TransformStamped, queue_size=4000)
    speed_sub = message_filters.Subscriber('bebop/states/ARDrone3/PilotingState/SpeedChanged', Ardrone3PilotingStateSpeedChanged, queue_size = 800)

    ts = message_filters.ApproximateTimeSynchronizer([att_sub, cmd_vel_sub, trans_sub, speed_sub], 10, 10.0)
    ts.registerCallback(sync_callback)

    rospy.loginfo("Starting data sync and export tool ...")
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    finally:
        rospy.loginfo("Smoothing ....")
        vic_vx_w_smoothed = smooth(np.array(time_l), np.array(vic_x_w),  smoothing_window_size, smoothing_order, 1)
        vic_vy_w_smoothed = smooth(np.array(time_l), np.array(vic_y_w),  smoothing_window_size, smoothing_order, 1)
        vic_vz_w_smoothed = smooth(np.array(time_l), np.array(vic_z_w),  smoothing_window_size, smoothing_order, 1)

        vic_ax_w_smoothed = smooth(np.array(time_l), np.array(vic_x_w),  smoothing_window_size, smoothing_order, 2)
        vic_ay_w_smoothed = smooth(np.array(time_l), np.array(vic_y_w),  smoothing_window_size, smoothing_order, 2)
        vic_az_w_smoothed = smooth(np.array(time_l), np.array(vic_z_w),  smoothing_window_size, smoothing_order, 2)

        rospy.loginfo("Writing to file ...")
        with open("out.csv", 'wb') as csvfile:
            # format <source>_<entity>_<coord>
            # vic: vicon, bebp: bebop, cvel: cmd_vel
            # w : world, b: bebop, n: normalized
            fn = ["time", "vic_x_w", "vic_y_w", "vic_z_w", "vic_roll_w", "vic_pitch_w", "vic_yaw_w", "vic_vx_w_smoothed", "vic_vy_w_smoothed", "vic_vz_w_smoothed", "vic_vx_b_smoothed", "vic_vy_b_smoothed", "vic_ax_w_smoothed", "vic_ay_w_smoothed", "vic_az_w_smoothed", "vic_ax_b_smoothed", "vic_ay_b_smoothed", "beb_vx_enu", "beb_vy_enu", "beb_vz_enu", "beb_vx_b", "beb_vy_b", "beb_roll_b", "beb_pitch_b", "beb_yaw_b", "cvel_rolln_b", "cvel_pitchn_b", "cvel_vzn_b", "cvel_vyawn_b"]
            csv_writer = csv.DictWriter(csvfile, fn)
            csv_writer.writeheader()
            for i in xrange(0, len(time_l)):
                phi = vic_yaw_w[i]
                vic_vx_b_smoothed.append(vic_vx_w_smoothed[i] * math.cos(phi) + vic_vy_w_smoothed[i] * math.sin(phi))
                vic_vy_b_smoothed.append(-vic_vx_w_smoothed[i] * math.sin(phi) + vic_vy_w_smoothed[i] * math.cos(phi))

                vic_ax_b_smoothed.append(vic_ax_w_smoothed[i] * math.cos(phi) + vic_ay_w_smoothed[i] * math.sin(phi))
                vic_ay_b_smoothed.append(-vic_ax_w_smoothed[i] * math.sin(phi) + vic_ay_w_smoothed[i] * math.cos(phi))

                csv_writer.writerow({"time": time_l[i],
                    "vic_x_w": vic_x_w[i],
                    "vic_y_w": vic_y_w[i],
                    "vic_z_w": vic_z_w[i],
                    "vic_roll_w": vic_roll_w[i],
                    "vic_pitch_w": vic_pitch_w[i],
                    "vic_yaw_w" : vic_yaw_w[i],
                    "vic_vx_w_smoothed": vic_vx_w_smoothed[i],
                    "vic_vy_w_smoothed": vic_vy_w_smoothed[i],
                    "vic_vz_w_smoothed": vic_vz_w_smoothed[i],
                    "vic_vx_b_smoothed": vic_vx_b_smoothed[i],
                    "vic_vy_b_smoothed": vic_vy_b_smoothed[i],
                    "vic_ax_w_smoothed": vic_ax_w_smoothed[i],
                    "vic_ay_w_smoothed": vic_ay_w_smoothed[i],
                    "vic_az_w_smoothed": vic_az_w_smoothed[i],
                    "vic_ax_b_smoothed": vic_ax_b_smoothed[i],
                    "vic_ay_b_smoothed": vic_ay_b_smoothed[i],
                    "beb_vx_enu": beb_vx_enu[i],
                    "beb_vy_enu": beb_vy_enu[i],
                    "beb_vz_enu": beb_vz_enu[i],
                    "beb_vx_b": beb_vx_b[i],
                    "beb_vy_b": beb_vy_b[i],
                    "beb_roll_b": beb_roll_b[i],
                    "beb_pitch_b": beb_pitch_b[i],
                    "beb_yaw_b": beb_yaw_b[i],
                    "cvel_rolln_b": cvel_rolln_b[i],
                    "cvel_pitchn_b": cvel_pitchn_b[i],
                    "cvel_vzn_b": cvel_vzn_b[i],
                    "cvel_vyawn_b": cvel_vyawn_b[i]})