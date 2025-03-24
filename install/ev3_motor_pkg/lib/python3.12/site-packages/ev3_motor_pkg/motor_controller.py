#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math
import sys

try:
    import ev3dev2.motor as motor
    SIMULATION_MODE = False
except ImportError:
    print("Warning: ev3dev2 not found, running in simulation mode")
    SIMULATION_MODE = True

class EV3MotorController(Node):
    def __init__(self):
        super().__init__('ev3_motor_controller')
        
        # Create subscribers for cmd_vel
        self.subscription = self.create_subscription(
            Twist, 
            'cmd_vel', 
            self.cmd_vel_callback, 
            10
        )
        
        # Motor configuration
        self.wheel_diameter = 0.056  # meters
        self.wheel_base = 0.135      # meters
        
        if not SIMULATION_MODE:
            try:
                # Configure motors
                self.left_motor = motor.LargeMotor(motor.OUTPUT_B)
                self.right_motor = motor.LargeMotor(motor.OUTPUT_C)
                
                # Reset motors
                self.left_motor.reset()
                self.right_motor.reset()
                
                self.get_logger().info("Motors initialized successfully")
            except Exception as e:
                self.get_logger().error(f"Failed to initialize motors: {str(e)}")
                sys.exit(1)
        
    def cmd_vel_callback(self, msg):
        """Convert ROS Twist messages to motor commands."""
        try:
            # Extract linear and angular velocities
            linear_speed = msg.linear.x   # m/s
            angular_speed = msg.angular.z  # rad/s
            
            # Calculate wheel speeds (m/s)
            left_speed = linear_speed - (angular_speed * self.wheel_base / 2.0)
            right_speed = linear_speed + (angular_speed * self.wheel_base / 2.0)
            
            # Convert to motor speeds (degrees per second)
            left_speed_dps = (left_speed * 360.0) / (math.pi * self.wheel_diameter)
            right_speed_dps = (right_speed * 360.0) / (math.pi * self.wheel_diameter)
            
            if not SIMULATION_MODE:
                # Apply speeds to motors
                self.left_motor.run_forever(speed_sp=int(left_speed_dps))
                self.right_motor.run_forever(speed_sp=int(right_speed_dps))
                
                self.get_logger().debug(
                    f"Motor speeds (dps) - Left: {left_speed_dps:.2f}, Right: {right_speed_dps:.2f}"
                )
            else:
                self.get_logger().info(
                    f"Simulation - Left: {left_speed_dps:.2f}, Right: {right_speed_dps:.2f}"
                )
                
        except Exception as e:
            self.get_logger().error(f"Error in cmd_vel_callback: {str(e)}")
    
    def stop_motors(self):
        """Safely stop the motors."""
        if not SIMULATION_MODE:
            try:
                self.left_motor.stop()
                self.right_motor.stop()
                self.get_logger().info("Motors stopped")
            except Exception as e:
                self.get_logger().error(f"Error stopping motors: {str(e)}")

def main(args=None):
    rclpy.init(args=args)
    controller = EV3MotorController()
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        controller.get_logger().info("Shutting down...")
    finally:
        controller.stop_motors()
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()