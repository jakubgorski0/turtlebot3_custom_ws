import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import threading

import sys, select, os
if sys.platform == 'win32':
    import msvcrt
else:
    import termios
    import tty

moveBindings = {
		'w':(1,0,0,0),
		'a':(0,0,0,1),
		'd':(0,0,0,-1),
		'x':(-1,0,0,0),
	       }
velBindings={
		'q':(0.025,0.0),
		'z':(-0.025,0.0),
        'e':(0.0,0.025),
		'c':(0.0,-0.025),
	      }
msg = """
Reading from the keyboard  and Publishing to Twist!
---------------------------
Moving around:
        w    
   a    s    d
        x    

anything else : stop

q/z : increase/decrease only linear speed by 0.025
e/c : increase/decrease only angular speed by 0.025

CTRL-C to quit
"""

MAX_VEL = 0.15
INIT_VEL = 0.1
MIN_VEL = 0.05

MAX_ANG_VEL = 1.0
INIT_ANG_VEL = 0.7
MIN_ANG_VEL = 0.4

class TeleopKey(Node):

    def __init__(self):
        
        # set terminal settings
        if sys.platform == 'win32':
            self.settings = None
        else:
            self.settings = termios.tcgetattr(sys.stdin)  

        super().__init__("custom_teleop_key")
        self.vel = INIT_VEL
        self.ang_vel = INIT_ANG_VEL
        self.x = 0
        self.y = 0
        self.z = 0
        self.th = 0
        self.status = 0
        self.get_logger().info("Started teleop node")
        self.pub_ = self.create_publisher(Twist, "cmd_vel", 10)
        
        self.spinner = threading.Thread(target=rclpy.spin, args=(self,))
        self.spinner.start()

        print(msg)
        print(self.vels(self.vel,self.ang_vel))

    def nodeLoop(self):
        try:
            while True:
                key = self.getKey()
                if key in moveBindings.keys():
                    self.x = moveBindings[key][0]
                    self.y = moveBindings[key][1]
                    self.z = moveBindings[key][2]
                    self.th = moveBindings[key][3]
                elif key in velBindings.keys():
                    self.vel = self.vel + velBindings[key][0]
                    self.ang_vel = self.ang_vel + velBindings[key][1]
                    self.vel = self.constrain(self.vel, MIN_VEL, MAX_VEL)
                    self.ang_vel = self.constrain(self.ang_vel, MIN_ANG_VEL, MAX_ANG_VEL)

                    print(self.vels(self.vel,self.ang_vel))
                    if(self.status == 14):
                        print(msg)
                    self.status = (self.status + 1) % 15

                else:
                    self.x = 0
                    self.y = 0
                    self.z = 0
                    self.th = 0
                    if (key == '\x03'):
                        break
                twist = self.createTwistMsg(self.x,self.y,self.z,
                            self.th,self.vel,self.ang_vel)
                self.pub_.publish(twist)

        except Exception as e:
            print(e)

        finally:
            twist = self.createTwistMsg(0.0,0.0,0.0,0.0,0.0,0.0)
            self.pub_.publish(twist)
            rclpy.shutdown()
            self.spinner.join()

            # restore terminal settings
            if sys.platform != 'win32':
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)

    def constrain(self,input_vel, lower_bound, higher_bound):
        if input_vel < lower_bound:
            input_vel = lower_bound
        if input_vel > higher_bound:
            input_vel = higher_bound

        return input_vel

    def createTwistMsg(self,x,y,z,th,vel,ang_vel):
        twist = Twist()
        twist.linear.x = x*vel 
        twist.linear.y = y*vel 
        twist.linear.z = z*vel
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = th*ang_vel
        return twist

    def getKey(self):
        if sys.platform == 'win32':
            # getwch() returns a string on Windows
            key = msvcrt.getwch()
        else:
            tty.setraw(sys.stdin.fileno())
            # sys.stdin.read() returns a string on Linux
            key = sys.stdin.read(1)
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key
        
    def vels(self,vel,ang_vel):
	    return "currently:\tspeed %s\tturn %s " % (vel,ang_vel)

def main():

    rclpy.init()

    test_teleop_key = TeleopKey()

    test_teleop_key.nodeLoop()
    
    test_teleop_key.destroy_node()


if __name__ == '__main__':
    main()