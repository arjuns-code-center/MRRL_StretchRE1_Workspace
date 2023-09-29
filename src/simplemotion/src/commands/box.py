# Arjun Viswanathan
# 3/6/23
# Autonomous command to move Stretch in a box

class auto_box:
    def __init__(self, robot):
        self.base = robot.base

        move = 0.5
        rotate = 1.65

        self.v = 10.0
        self.a = 5.0

        self.timeout = 1

        print("Moving robot in a box")

        for i in range(4):
            self.move_base(move, wait=True)
            robot.push_command()

            self.rotate_base(rotate, wait=True)
            robot.push_command()

        print("Finished box!")

    def move_base(self, x, wait=False):
        # Use distance
        self.base.translate_by(x_m=x, v_m=self.v, a_m=self.a)

        if wait:
            self.base.wait_until_at_setpoint(self.timeout)

    def rotate_base(self, theta, wait=False):
        # Use distance
        self.base.rotate_by(x_r=theta, v_r=self.v, a_r=self.a)

        if wait:
            self.base.wait_until_at_setpoint(self.timeout)