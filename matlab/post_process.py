#!/usr/bin/env python

import sys
import os
import rosbag
from geometry_msgs.msg import Twist, TwistStamped

# from: https://gist.github.com/vladignatyev/06860ec2040cb497f0f3
def progress(count, total, status=''):
    bar_len = 60
    filled_len = int(round(bar_len * count / float(total)))

    percents = round(100.0 * count / float(total), 1)
    bar = '=' * filled_len + '-' * (bar_len - filled_len)

    sys.stdout.write('[%s] %s%s ...%s\r' % (bar, percents, '%', status))
    sys.stdout.flush()  # As suggested by Rom Ruben (see: http://stackoverflow.com/questions/3173320/text-progress-bar-in-the-console/27871113#comment50529068_27871113)


def main():
    if not len(sys.argv) == 2:
        print "Usage: ./post_process.py <input.bag>"
        sys.exit(1)

    topics=["/bebop/states/ARDrone3/PilotingState/AttitudeChanged", "/bebop/cmd_vel", "/vicon/bebop_blue_en/bebop_blue_en", "/bebop/states/ARDrone3/PilotingState/SpeedChanged"]

    input_bag = sys.argv[1]
    output_bag = os.path.splitext(os.path.basename(input_bag))[0] + "_processed.bag"

    cmd_vel_stamped = TwistStamped()
    cmd_vel_stamped.header.frame_id = "base_link"

    print "Output: ", output_bag

    i = 0
    try:
        i_bag = rosbag.Bag(input_bag)
        o_bag = rosbag.Bag(output_bag, "w")
        total_msgs =  i_bag.get_message_count(topics)
        for topic, msg, t in i_bag.read_messages(topics):
            o_bag.write(topic, msg, t)
            if topic == "/bebop/cmd_vel":
                cmd_vel_stamped.header.stamp = t
                cmd_vel_stamped.twist = msg
                o_bag.write("/bebop/cmd_vel_stamped", cmd_vel_stamped, t)
            i += 1
            progress(i, total_msgs, output_bag)
    finally:
        o_bag.close()
        i_bag.close()

if __name__ == "__main__":
    main()
