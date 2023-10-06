import stretch_body.robot as sb
import rospy
from geometry_msgs.msg import Twist
import sys, tty, termios

def getkeystroke():
    fd=sys.stdin.fileno()
    old_settings=termios.tcgetattr(fd)

    try:
        tty.setraw(sys.stdin.fileno())
        ch=sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd,termios.TCSADRAIN,old_settings)

    return ch

# def callback(r, v, a):
#     while True:
#         key = getkeystroke()

#         r.base.translate_by(x_m=0.15, v_m=v, a_m=a)
#         r.push_command()

#         if key == 't' or key == 'T':
#             r.stop()
#             rospy.signal_shutdown('end of execution')

if __name__ == '__main__':
    rospy.init_node('move_forward')
    pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)

    cmd = Twist()
    cmd.linear.x = 1.0

    while True:
        pub.publish(cmd)
        key = getkeystroke()

        if key == 't' or key == 'T':
            rospy.signal_shutdown('end of execution')
    
    # r = sb.Robot()
    # r.startup()
    v = 10.0
    a = 5.0

    # callback(v, a)
    # rospy.spin()

# import stretch_body.robot as sb
# import rospy
# import sys, tty, termios

# rospy.init_node('move_forward')

# r = sb.Robot()
# r.startup()

# v = 10.0
# a = 5.0

# b = r.base

# def getkeystroke():
#     fd=sys.stdin.fileno()
#     old_settings=termios.tcgetattr(fd)

#     try:
#         tty.setraw(sys.stdin.fileno())
#         ch=sys.stdin.read(1)
#     finally:
#         termios.tcsetattr(fd,termios.TCSADRAIN,old_settings)

#     return ch

# while True:
#     key = getkeystroke()

#     r.base.translate_by(x_m=0.15, v_m=v, a_m=a)

#     if key == 't' or key == 'T':
#         break

# r.stop()
# rospy.signal_shutdown('end of execution')