import rclpy
from std_msgs.msg import Bool
import serial
import time

def main():
    rclpy.init()
    node = rclpy.create_node('ros_arduino_communication')
    pub = node.create_publisher(Bool, 'motor_control', 10)

    # Establish a serial connection to the Arduino
    arduino = serial.Serial('/dev/ttyACM1', 9600)  # Replace with the correct serial port and baud rate

    try:
        while rclpy.ok():
            user_input = input("Enter =  'F' to turn on the motor, 'S' to turn it off, or 'q' to quit: ")

            if user_input == 'F':
                msg = Bool()
                msg.data = True
                pub.publish(msg)
                node.get_logger().info('Published motor control message (ON)')
                arduino.write(b'F')  # Send '1' to the Arduino to turn on the motor
            elif user_input == 'S':
                msg = Bool()
                msg.data = False
                pub.publish(msg)
                node.get_logger().info('Published motor control message (OFF)')
                arduino.write(b'S')  # Send '0' to the Arduino to turn off the motor
            elif user_input == 'q':
                break

            time.sleep(1)  # Delay to allow time for the Arduino to respond

    except KeyboardInterrupt:
        pass

    arduino.close()  # Close the serial connection
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
# # computer_node.py
# import rclpy
# from std_msgs.msg import Int32

# def main():
#     rclpy.init()
#     node = rclpy.create_node('motor_control_computer_node')
#     pub = node.create_publisher(Int32, 'motor_control', 10)
#     rate = node.create_rate(1)  # Adjust the rate as needed

#     while rclpy.ok():
#         # Send a command to control the motor (1 for ON, 0 for OFF)
#         motor_command = Int32()
#         motor_command.data = 1  # Change this value to control the motor
#         pub.publish(motor_command)
#         rate.sleep()

#     rclpy.spin()

# if __name__ == '__main__':
#     main()

