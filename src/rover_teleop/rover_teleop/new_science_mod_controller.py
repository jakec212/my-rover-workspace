#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from example_interfaces.msg import Float32MultiArray

class NewScienceModController(Node):
    def __init__(self):
        super().__init__('new_science_mod_controller')
        
        # Subscriber to joy topic
        self.joy_sub = self.create_subscription(
            Joy,
            '/joy',
            self.joy_callback,
            10
        )
        
        # Publisher for Float32MultiArray message (matches Teensy expectation)
        self.science_pub = self.create_publisher(
            Float32MultiArray,  # CHANGED: Using Float32MultiArray
            '/new_science_module/commands',
            10
        )

         # ===== DEBOUNCER LOGIC FOR STEPPER MOTOR BUTTONS: Track previous button states for edge detection =====
        self.prev_button0 = 0
        self.prev_button1 = 0
        self.prev_button4 = 0
        self.prev_button5 = 0
        
        self.get_logger().info('New Science Module Controller Started')
        
    
    def joy_callback(self, msg):
        cmd = Float32MultiArray()  # CHANGED: Using Float32MultiArray
        cmd.data = [1500.0] * 4
        
        # =========== AUGER CONTROL LOGIC data[0] =================
        axes2 = msg.axes[2]  # CCW trigger (range: -1.0 to 1.0)
        axes5 = msg.axes[5]  # CW trigger (range: -1.0 to 1.0) - FIXED axis number
        
        # Triggers usually rest at 1.0 and go to -1.0 when pressed
        # Consider "pressed" if value is less than 0.5
        TRIGGER_THRESHOLD = 0.5
        
        axes2_pressed = axes2 < TRIGGER_THRESHOLD
        axes5_pressed = axes5 < TRIGGER_THRESHOLD
        
        # Both triggers pressed
        if axes2_pressed and axes5_pressed:
            cmd.data[0] = 1500.0
        
        # Only axes[2] (CCW) pressed
        elif axes2_pressed:
            # Map from -1.0 to 1.0 → 0 to 200
            normalized = -1*(axes2 - 1) / 2  # 0 to 1
            mapped_value = normalized * 200.0  # 0 to 200
            cmd.data[0] = 1500.0 - mapped_value
        
        # Only axes[5] (CW) pressed
        elif axes5_pressed:
            normalized = -1*(axes5 - 1) / 2
            mapped_value = normalized * 200.0
            cmd.data[0] = 1500.0 + mapped_value
        
        else:
            cmd.data[0] = 1500.0

        # ========== ACTUATOR CONTROL (data[3]) ==========
        axes7 = msg.axes[7]  # Actuator control (normalized: -1.0 to 1.0)
        
        ACTUATOR_SPEED = 300.0  # Adjust this constant to change speed
        
        if axes7 == -1.0:
            cmd.data[3] = 1500.0 - ACTUATOR_SPEED  # Retract (1200)
        elif axes7 == 1.0:
            cmd.data[3] = 1500.0 + ACTUATOR_SPEED  # Extend (1800)
        else:
            cmd.data[3] = 1500.0  # Idle/stopped

        # ========== CAROUSEL CONTROL (data[1]) (SAMPLING PORTION)==========
        button4 = msg.buttons[4]  # CCW rotation
        button5 = msg.buttons[5]  # CW rotation
        
        STEPS_PER_360_DEGREES = 1600  #       ---     This is the line to change how much the carousel moves
        carousel_steps = 450/360 * STEPS_PER_360_DEGREES    #should be 2000 steps 
        
        # Check for RISING EDGE (button just pressed, not held)
        button4_rising_edge = button4 and not self.prev_button4
        button5_rising_edge = button5 and not self.prev_button5
        
        if button4_rising_edge and button5_rising_edge:
            # Both buttons pressed simultaneously - do nothing
            cmd.data[1] = 0.0
        elif button4_rising_edge:
            # Button 4 just pressed - rotate -60 degrees (CCW)
            cmd.data[1] = -carousel_steps
            self.get_logger().info('Carousel CCW -60°')
        elif button5_rising_edge:
            # Button 5 just pressed - rotate +60 degrees (CW)
            cmd.data[1] = carousel_steps
            self.get_logger().info('Carousel CW +60°')
        else:
            # No new button press - send idle command
            cmd.data[1] = 0.0
        
        # Update previous button states for next callback
        self.prev_button4 = button4
        self.prev_button5 = button5

        # ========== PLUNGER CONTROL (data[2]) WITH EDGE DETECTION (PLUNGER OF THE WHOLE MODULE)==========
        button0 = msg.buttons[0]  # +10 degrees
        button1 = msg.buttons[1]  # -10 degrees
        
        PLUNGER_INCHES_DROPPED = 23.077 * STEPS_PER_360_DEGREES  #  Steps/Inch of travel      ---     This Section will change the steps for the plunger for the whole piece
        
        # Check for RISING EDGE
        button0_rising_edge = button0 and not self.prev_button0
        button1_rising_edge = button1 and not self.prev_button1
        
        if button0_rising_edge and button1_rising_edge:
            # Both buttons pressed simultaneously - do nothing
            cmd.data[2] = 0.0
        elif button0_rising_edge:
            # Button 0 just pressed - rotate +10 degrees
            cmd.data[2] = PLUNGER_INCHES_DROPPED
            self.get_logger().info('Plunger +10°')
        elif button1_rising_edge:
            # Button 1 just pressed - rotate -10 degrees
            cmd.data[2] = -PLUNGER_INCHES_DROPPED
            self.get_logger().info('Plunger -10°')
        else:
            # No new button press - send idle command
            cmd.data[2] = 0.0
        
        # Update previous button states for next callback
        self.prev_button0 = button0
        self.prev_button1 = button1
        
        # Log for debugging
        self.get_logger().info(
            f'Auger: {cmd.data[0]:.2f}, Carousel: {cmd.data[1]:.2f}, '
            f'Stepper: {cmd.data[2]:.2f}, Actuator: {cmd.data[3]:.2f}'
        )
        
        self.science_pub.publish(cmd)
        

def main(args=None):
    rclpy.init(args=args)
    node = NewScienceModController()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()