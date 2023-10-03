import stretch_body.robot as sb
import rospy
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

def callback(r, v, a):
    while True:
        key = getkeystroke()

        r.base.rotate_by(x_r=0.15, v_m=v, a_m=a)
        r.push_command()

        if key == 't' or key == 'T':
            r.stop()
            rospy.signal_shutdown('end of execution')

if __name__ == '__main__':
    rospy.init_node('turn_left')

    r = sb.Robot()
    r.startup()
    v = 10.0
    a = 5.0

    callback(r, v, a)
    rospy.spin()