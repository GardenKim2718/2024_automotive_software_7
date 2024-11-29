#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from ad_msgs.msg import VehicleCommand
# import tf_transformations
# import random
import numpy as np
# import math
try:
    import pygame
    from pygame.locals import K_ESCAPE
    from pygame.locals import K_DOWN
    from pygame.locals import K_LEFT
    from pygame.locals import K_RIGHT
    from pygame.locals import K_UP
    from pygame.locals import K_i
    from pygame.locals import K_j
    from pygame.locals import K_k
    from pygame.locals import K_l
    from pygame.locals import K_q
    from pygame.locals import K_z
    from pygame.locals import K_w
    from pygame.locals import K_x
    from pygame.locals import K_e
    from pygame.locals import K_c
    from pygame.locals import K_SPACE
except ImportError:
    raise RuntimeError('cannot import pygame, make sure pygame package is installed. Run "$pip install pygame"')

msg = """
Control Your Vehicle!
---------------------------
Moving around:
        i       |   ARROW
   j    k    l  |

q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear accel by 10%
e/c : increase/decrease only angular speed by 10%
k: brake
space key: steering 0

CTRL-C to quit
"""


class Keyboard(Node):
    def __init__(self):
        super().__init__('manual_input')
        self.movement_publisher = self.create_publisher(VehicleCommand, 'manual_input', 10)
        self.ros_rate = 50
        self.kAccel = 5.0
        self.kAngle = 0.5
        self.PI = 3.14159265358979323846
        # self.use_manual = self.declare_parameter("ego/autonomous_driving/autonomous_driving/use_manual_inputs", True).get_parameter_value().bool_value
        self.use_manual = True
        
    def vels(self, kAccel, kAngle):
        return "accel constant: %s\t angle constant: %s\t" % (self.kAccel, self.kAngle)

    def run(self):
        rate = self.create_rate(self.ros_rate)

        accel = 0
        frontAngle = 0
        brake = 0

        print(msg)
        print(self.vels(self.kAccel, self.kAngle))

        while rclpy.ok():
            
            # Transfer keyboard state in pygame -> msgs
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    return True
                elif event.type == pygame.KEYUP:
                    if self._is_quit_shortcut(event.key):
                        return True

            keys = pygame.key.get_pressed()

            if (keys[K_UP] and keys[K_RIGHT]) or (keys[K_i] and keys[K_l]):
                accel += self.kAccel
                accel = sorted([0, accel, 100])[1]
                frontAngle -= self.kAngle
                frontAngle = sorted([-45, frontAngle, 45])[1]
                print("accel : %s %%\t frontAngle : %s degree\t" %
                      (accel, frontAngle))
            elif (keys[K_UP] and keys[K_LEFT]) or (keys[K_i] and keys[K_j]):
                accel += self.kAccel
                accel = sorted([0, accel, 100])[1]
                frontAngle += self.kAngle
                frontAngle = sorted([-45, frontAngle, 45])[1]
                print("accel : %s %%\t frontAngle : %s degree\t" %
                      (accel, frontAngle))
            elif (keys[K_DOWN] and keys[K_RIGHT]) or (keys[K_k] and keys[K_l]):
                brake += self.kAccel
                brake = sorted([0, brake, 100])[1]
                frontAngle -= self.kAngle
                frontAngle = sorted([-45, frontAngle, 45])[1]
                print("brake : %s %%!!!!!!!!!!\t " % (brake))

            elif (keys[K_DOWN] and keys[K_LEFT]) or (keys[K_k] and keys[K_j]):
                brake += self.kAccel
                brake = sorted([0, brake, 100])[1]
                frontAngle += self.kAngle
                frontAngle = sorted([-45, frontAngle, 45])[1]
                print("brake : %s %%!!!!!!!!!!\t " % (brake))

            elif keys[K_UP] or keys[K_i]:
                frontAngle /= 1.05
                accel += self.kAccel
                accel = sorted([0, accel, 100])[1]
                print("accel : %s %%\t frontAngle : %s degree\t" %
                      (accel, frontAngle))
            elif keys[K_LEFT] or keys[K_j]:
                frontAngle += self.kAngle
                frontAngle = sorted([-45, frontAngle, 45])[1]
                print("accel : %s %%\t frontAngle : %s degree\t" %
                      (accel, frontAngle))
            elif keys[K_RIGHT] or keys[K_l]:
                frontAngle -= self.kAngle
                frontAngle = sorted([-45, frontAngle, 45])[1]
                print("accel : %s %%\t frontAngle : %s degree\t" %
                      (accel, frontAngle))
            elif keys[K_DOWN] or keys[K_k]:
                frontAngle /= 1.05
                brake += self.kAccel
                brake = sorted([0, brake, 100])[1]
                print("brake : %s %%!!!!!!!!!!\t " % (brake))

            elif keys[K_q]:
                pass

            elif keys[K_z]:
                pass

            elif keys[K_w]:
                self.kAccel = self.kAccel * 1.1

            elif keys[K_x]:
                self.kAccel = self.kAccel * 0.9

            elif keys[K_e]:
                self.kAngle = self.kAngle * 1.1

            elif keys[K_c]:
                self.kAngle = self.kAngle * 0.9
            elif keys[K_SPACE]:
                frontAngle = 0.0
                brake = 100.0
            else:
                accel -= self.kAccel*2.0
                accel = sorted([0, accel, 100])[1]
                brake = 0.0
                frontAngle /= 1.05

                
            input_msg = VehicleCommand()
            input_msg.accel = accel / 100.0
            input_msg.steering = frontAngle / 180.0 * self.PI
            input_msg.brake = brake / 100.0
            self.movement_publisher.publish(input_msg)
            # rate.sleep()
            rclpy.spin_once(self, timeout_sec=0.05)
    
    @staticmethod
    def _is_quit_shortcut(key):
        return (key == K_ESCAPE)


def main(args=None):
    rclpy.init(args=args)
    key = Keyboard()
    print("use_manual : %s" % (key.use_manual))
    if key.use_manual:
        pygame.init()
        pgscreen = pygame.display.set_mode((1, 1))
        pygame.display.set_caption('keyboard')
        key.run()

    rclpy.shutdown()

if __name__ == '__main__':
    main()
