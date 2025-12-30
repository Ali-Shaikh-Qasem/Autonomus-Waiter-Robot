import tkinter as tk
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import math

class RestaurantRobotNavigation(Node):
    def __init__(self):
        super().__init__('restaurant_robot_navigation')
        self.publisher_ = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.table_poses = {
            1: (1.0, 2.0, 0.0),  # Table 1 (x, y, orientation)
            2: (2.5, 3.0, 0.0),  # Table 2
            3: (4.0, 1.5, 1.57), # Table 3 (90 degrees rotation)

        }

    def set_goal(self, table_number):
        if table_number in self.table_poses:
            x, y, orientation = self.table_poses[table_number]
            goal_msg = PoseStamped()
            goal_msg.header.stamp = self.get_clock().now().to_msg()
            goal_msg.header.frame_id = 'map'  # Change if using another frame
            
            goal_msg.pose.position.x = x
            goal_msg.pose.position.y = y
            goal_msg.pose.position.z = 0.0  # 2D navigation, so z is 0

            # Convert orientation to quaternion (yaw only)
            q = self.yaw_to_quaternion(orientation)
            goal_msg.pose.orientation.x = q[0]
            goal_msg.pose.orientation.y = q[1]
            goal_msg.pose.orientation.z = q[2]
            goal_msg.pose.orientation.w = q[3]

            self.publisher_.publish(goal_msg)
            self.get_logger().info(f"Published goal for Table {table_number} at ({x}, {y})")

    def yaw_to_quaternion(self, yaw):
        """ Convert yaw (in radians) to quaternion (for 2D) """
        q = [0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)]
        return q


class GUI(tk.Tk):
    def __init__(self, navigation_node):
        super().__init__()
        self.navigation_node = navigation_node
        self.title("Restaurant Table Navigation")
        
        self.create_buttons()

    def create_buttons(self):
        """ Create buttons for each table """
        for table_number in self.navigation_node.table_poses:
            button = tk.Button(self, text=f"Table {table_number}", command=lambda t=table_number: self.on_button_press(t))
            button.pack(padx=20, pady=10)

    def on_button_press(self, table_number):
        """ Handle button press, send robot to corresponding table """
        self.navigation_node.set_goal(table_number)
        print(f"Going to Table {table_number}")


def main(args=None):
    rclpy.init(args=args)

    # Create the robot navigation node
    navigation_node = RestaurantRobotNavigation()

    # Create the GUI
    gui = GUI(navigation_node)
    
    # Run the Tkinter main loop in a separate thread
    gui.after(100, gui.mainloop)  # Avoid blocking rclpy

    # Spin ROS 2 to keep the robot navigation node running
    rclpy.spin(navigation_node)
    rclpy.shutdown()

if __name__ == "__main__":
    print("Script is running...")

    main()
