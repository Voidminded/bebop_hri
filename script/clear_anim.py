#!/usr/bin/env python
import rospy
from autonomy_leds_msgs.msg import Animation, Keyframe, Command
from std_msgs.msg import ColorRGBA
import time, copy

num_leds = 9

def main():
    display_pub = rospy.Publisher('leds/display', Keyframe, queue_size=10)
    anim_pub = rospy.Publisher('leds/animation', Animation, queue_size=10)
    set_pub = rospy.Publisher('leds/set', Command, queue_size=10)

    rospy.init_node('led_anim_clear', anonymous = True)
    time.sleep(0.5)

    keyframe_msg = Keyframe()
    command_msg = Command()
    color_msg = ColorRGBA()
    anim_msg = Animation()
    
    rospy.loginfo("Single frame test ...")
    keyframe_msg.color_pattern = []
    color_msg.r = 0.0
    color_msg.g = 0.0
    color_msg.b = 0.0
    keyframe_msg.color_pattern.append(copy.deepcopy(color_msg))
    display_pub.publish(keyframe_msg)
    time.sleep(0.5)

    rospy.loginfo("Clearing the animation...")    
    anim_msg.iteration_count = 1
    anim_pub.publish(anim_msg)
    time.sleep(1)

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
