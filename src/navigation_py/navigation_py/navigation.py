import rclpy
from rclpy.node import Node
from navigation_py.util import update_screen
from std_msgs.msg import Float64MultiArray
import numpy as np
import random
import curses

screen = curses.initscr()

class DroneNode(Node):
    def __init__(self):
        super().__init__('drone_node')
        #screen is 400x400
        #must land directly on target_pos

        #define the target and drone points
        self.target_pos = np.array([346.41, 200])
        self.current_pos = np.array([0.0,0.0])

        #defines the publishers for these points
        self.pos_publisher = self.create_publisher(Float64MultiArray, 
            'currentpos', 10)
        self.target_publisher = self.create_publisher(Float64MultiArray, 
            'targetpos', 10)

        #updates the points every 0.5 seconds
        timer_period = 0.5
        self.timer_ = self.create_timer(timer_period, self.timer_callback)

        #subscribes to the velocity (published by the solution node)
        self.vel_subscription = self.create_subscription(Float64MultiArray,
            'drone_vel',
            self.update_position,
            10)
        
        self.success = False
        self.count = 0

    def run_wind(self):
        self.current_pos[0] = (max(0, round(self.current_pos[0]
            +random.uniform(-1.99,1.99),2)))
        self.current_pos[1] = (max(0, round(self.current_pos[1]
            +random.uniform(-1.99,2.0),2)))

    def timer_callback(self):
        self.run_wind()
        #stores info into sendable objects
        pos_msg = Float64MultiArray()
        pos_msg.data = [float(self.current_pos[0]), float(self.current_pos[1])]
        targ_msg = Float64MultiArray()
        targ_msg.data = [float(self.target_pos[0]), float(self.target_pos[1])]

        if self.success == False:
        #publishes the data
            self.pos_publisher.publish(pos_msg)
            self.target_publisher.publish(targ_msg)
            update_screen(self.current_pos, self.target_pos, screen)
        else:
            curses.nocbreak()
            curses.echo()
            curses.endwin()
            self.get_logger().info("Completed")

    def update_position(self, msg):
        # round to prevent impossible to align float precision
        x_vel = round(msg.data[0], 2)
        y_vel = round(msg.data[1], 2)

        if (x_vel) > 20:
            x_vel = 20
        if (x_vel) < -20:
            x_vel = -20
        if (y_vel > 20):
            y_vel = 20
        if (y_vel < -20):
            y_vel = -20
        
        # update current_position
        x_pos = self.current_pos[0]+x_vel
        y_pos = self.current_pos[1]+y_vel
        x_pos = min(399, max(0, x_pos))
        y_pos = min(399, max(0, y_pos))
        self.current_pos = np.array([x_pos, y_pos])

        #check for equality
        if (round(self.target_pos[0], 2) == round(x_pos,2) and 
            round(self.target_pos[1], 2) == round(y_pos,2)):
            self.count+=1
            if (self.count == 4):
                self.success = True
        else:
            self.count = 0
        update_screen(self.current_pos, self.target_pos, screen)
        screen.refresh()
        return
    
def main(args=None):
    rclpy.init()

    drone_node = DroneNode()

    rclpy.spin(drone_node)

    drone_node.destroy_node()
    rclpy.shutdown()
    