"""
different_3d_parabolas.py
Various 3D parabolic cost functions you can use with x1, x2
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Float64MultiArray, Bool
import numpy as np
import math

class Advanced3DParabolicTest(Node):
    def __init__(self):
        super().__init__('advanced_parabolic_test')
        
        # Choose your 3D function type
        self.function_type = 1  # Change this to select different functions (1-6)
        
        # Optimal points for different functions
        self.target_x1 = 3.0
        self.target_x2 = -2.0
        
        # ROS setup
        self.sub = self.create_subscription(Float64MultiArray, '/bayes_opt/params_to_test', self.callback, 10)
        self.pub_result = self.create_publisher(Float64, '/bayes_opt/experiment_result', 10)
        self.pub_allow = self.create_publisher(Bool, '/bayes_opt/allow_step', 10)
        
        self.iteration = 0
        self.print_function_info()
        
    def print_function_info(self):
        """Print information about the selected function"""
        functions = {
            1: "Standard 3D Paraboloid: f = (x1-a)² + (x2-b)²",
            2: "Elliptic Paraboloid: f = 2(x1-a)² + 0.5(x2-b)²",
            3: "Rotated Paraboloid: f = (x1-a)² + (x2-b)² + 0.5(x1-a)(x2-b)",
            4: "Steep Paraboloid: f = 5(x1-a)² + 5(x2-b)²",
            5: "Flat Paraboloid: f = 0.1(x1-a)² + 0.1(x2-b)²",
            6: "Saddle-like Paraboloid: f = (x1-a)² - 0.5(x2-b)² + 2(x2-b)⁴"
        }
        print("="*60)
        print(f"3D PARABOLIC FUNCTION TYPE {self.function_type}")
        print(functions.get(self.function_type, "Unknown"))
        print(f"Optimal point: ({self.target_x1}, {self.target_x2})")
        print("="*60 + "\n")
    
    def compute_3d_parabola(self, x1, x2):
        """
        Different 3D parabolic functions
        All create 3D surfaces with x1, x2 as inputs
        """
        # Center the coordinates
        dx = x1 - self.target_x1
        dy = x2 - self.target_x2
        
        if self.function_type == 1:
            # Standard circular paraboloid (bowl)
            return dx**2 + dy**2
            
        elif self.function_type == 2:
            # Elliptic paraboloid (elongated bowl)
            return 2*dx**2 + 0.5*dy**2
            
        elif self.function_type == 3:
            # Rotated paraboloid (tilted bowl)
            return dx**2 + dy**2 + 0.5*dx*dy
            
        elif self.function_type == 4:
            # Steep paraboloid (narrow bowl)
            return 5*dx**2 + 5*dy**2
            
        elif self.function_type == 5:
            # Flat paraboloid (shallow bowl)
            return 0.1*dx**2 + 0.1*dy**2
            
        elif self.function_type == 6:
            # More complex 3D shape (still has minimum at target)
            return dx**2 - 0.5*dy**2 + 2*dy**4
            
        else:
            # Default to standard
            return dx**2 + dy**2
    
    def callback(self, msg):
        self.iteration += 1
        
        # Get x1 and x2
        x1 = msg.data[0]
        x2 = msg.data[1]
        
        # Compute 3D parabolic cost
        cost = self.compute_3d_parabola(x1, x2)
        
        # Add small noise for realism
        cost += np.random.normal(0, 0.001)
        
        # Calculate distance from optimum
        distance = math.sqrt((x1 - self.target_x1)**2 + (x2 - self.target_x2)**2)
        
        print(f"Iter {self.iteration}: x1={x1:.3f}, x2={x2:.3f}, "
              f"3D_cost={cost:.4f}, distance={distance:.4f}")
        
        if distance < 0.01:
            print(f"🎯 FOUND OPTIMUM of 3D surface! ({x1:.4f}, {x2:.4f})")
        
        # Send 3D cost back
        self.pub_result.publish(Float64(data=cost))
        self.pub_allow.publish(Bool(data=True))

def main():
    rclpy.init()
    node = Advanced3DParabolicTest()
    rclpy.spin(node)

if __name__ == '__main__':
    main()