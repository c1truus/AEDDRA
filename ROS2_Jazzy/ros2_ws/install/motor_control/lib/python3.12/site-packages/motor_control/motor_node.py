#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import socket
import struct
import serial
import time

class RobotNode(Node):
    def __init__(self):
        super().__init__('motor_node')
        self.get_logger().info("ESP32 Robot Drive Node Started")

        # Initialize serial port
        self.serial_port = None
        try:
            self.serial_port = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
            self.serial_port.reset_input_buffer()
            self.serial_port.reset_output_buffer()
            self.get_logger().info("Serial port initialized successfully")
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to open serial port: {e}")
            raise

        # Initialize UDP socket
        self.udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.udp_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.udp_socket.settimeout(0.5)
        try:
            self.udp_socket.bind(('172.20.10.14', 12345))
            self.get_logger().info("UDP socket bound to 172.20.10.5:12345")
        except socket.error as e:
            self.get_logger().error(f"Failed to bind UDP socket: {e}")
            raise

        # Initialize control variables
        self.inputs = {
            'left_x': 2048, 'left_y': 2048, 'right_x': 2048, 'right_y': 2048,
            'up': False, 'down': False, 'left': False, 'right': False,
            'a': False, 'b': False, 'x': False, 'y': False,
            'pot_val': 2048, 'encoder_val': 0
        }
        self.last_encoder_val = 0
        self.led_state = 0
        self.button_states = {key: False for key in ['down', 'a', 'b', 'x', 'y']}
        self.speed_mode = False  # False = slow (±1200), True = fast (±2000)
        self.servos = {'s1': 55, 's2': 90, 's3': 90, 's4': 90}
        self.selected_servo = None
        self.paused = False
        self.last_udp_receive = time.time()
        self.last_serial_success = time.time()

        # Joystick parameters
        self.joystick_center = 2048
        self.joystick_max_deviation = 2048.0
        self.deadzone = 200

        # Timers
        self.timer = self.create_timer(0.05, self.control_loop)  # 20 Hz
        self.led_timer = self.create_timer(5.0, self.toggle_led)

    def toggle_led(self):
        self.led_state = 1 - self.led_state
        self.get_logger().info(f"LED toggled to {self.led_state}")

    def handle_buttons(self):
        if self.inputs['down'] and not self.button_states['down']:
            self.speed_mode = not self.speed_mode
            self.button_states['down'] = True
            speed = "fast (±2000)" if self.speed_mode else "slow (±1200)"
            self.get_logger().info(f"Speed mode toggled to {speed}")
        elif not self.inputs['down']:
            self.button_states['down'] = False

        servo_buttons = {'a': 's1', 'b': 's2', 'x': 's3', 'y': 's4'}
        for btn, servo in servo_buttons.items():
            if self.inputs[btn] and not self.button_states[btn]:
                self.button_states[btn] = True
                self.selected_servo = servo
                self.get_logger().info(f"{btn.upper()} button pressed: Servo {servo} selected")
            elif not self.inputs[btn]:
                self.button_states[btn] = False

    def handle_encoder(self):
        if self.selected_servo and self.inputs['encoder_val'] != self.last_encoder_val:
            delta = self.inputs['encoder_val'] - self.last_encoder_val
            self.servos[self.selected_servo] = max(0, min(180, self.servos[self.selected_servo] + delta * 3))
            self.get_logger().info(
                f"Servo {self.selected_servo} adjusted to {self.servos[self.selected_servo]} degrees (delta={delta})"
            )
            self.last_encoder_val = self.inputs['encoder_val']
            self.inputs['encoder_val'] = 0

    def control_loop(self):
        try:
            # Read UDP packet
            try:
                data, addr = self.udp_socket.recvfrom(20)
                self.inputs.update(zip(
                    ['left_x', 'left_y', 'right_x', 'right_y', 'up', 'down', 'right', 'left',
                     'a', 'b', 'x', 'y', 'pot_val', 'encoder_val'],
                    struct.unpack('<4h8bhh', data)
                ))
                self.last_udp_receive = time.time()
                self.get_logger().info(f"Received UDP packet from {addr}, size: {len(data)} bytes")
            except socket.timeout:
                self.get_logger().warn("UDP receive timeout")
                if time.time() - self.last_udp_receive > 1.0:
                    self.get_logger().error("No UDP packets received for 1 second, triggering emergency stop")
                    self._emergency_stop()
                return
            except socket.error as e:
                self.get_logger().error(f"UDP receive error: {e}")
                return

            # Handle inputs
            self.handle_buttons()
            self.handle_encoder()

            if self.paused:
                self._emergency_stop()
                return

            # Normalize joystick inputs
            def normalize_axis(raw_value):
                offset = raw_value - self.joystick_center
                return 0.0 if abs(offset) < self.deadzone else offset / self.joystick_max_deviation

            Vx_joy = normalize_axis(self.inputs['left_y'])
            Vy_joy = normalize_axis(self.inputs['left_x'])
            Wz_joy = normalize_axis(self.inputs['right_y'])

            # Speed mode scaling
            max_speed = 2000 if self.speed_mode else 1200

            # Mecanum kinematics
            raw_m1 = Vx_joy + Vy_joy + Wz_joy
            raw_m2 = Vx_joy - Vy_joy + Wz_joy
            raw_m3 = Vx_joy - Vy_joy - Wz_joy
            raw_m4 = Vx_joy + Vy_joy - Wz_joy

            # Normalize to [-max_speed, max_speed]
            max_raw = max(abs(raw_m1), abs(raw_m2), abs(raw_m3), abs(raw_m4), 1.0)
            scale = max_speed / max_raw

            # Map to [-max_speed, max_speed] as int16_t
            m1 = int(raw_m1 * scale)
            m2 = int(raw_m2 * scale)
            m3 = int(raw_m3 * scale)
            m4 = int(raw_m4 * scale)

            # Clamp servo angles
            for servo in self.servos:
                self.servos[servo] = max(0, min(180, self.servos[servo]))

            # Send serial command
            max_retries = 3
            for attempt in range(max_retries):
                try:
                    packet = struct.pack('<B4h4B', self.led_state, m1, m2, m3, m4,
                                         self.servos['s1'], self.servos['s2'],
                                         self.servos['s3'], self.servos['s4'])
                    self.serial_port.write(packet)
                    self.last_serial_success = time.time()
                    self.get_logger().info(
                        f"Sent serial: LED={self.led_state}, m1={m1}, m2={m2}, m3={m3}, m4={m4}, "
                        f"s1={self.servos['s1']}, s2={self.servos['s2']}, s3={self.servos['s3']}, "
                        f"s4={self.servos['s4']}, Vx={Vx_joy:.2f}, Vy={Vy_joy:.2f}, Wz={Wz_joy:.2f}, "
                        f"Speed={max_speed}"
                    )
                    response = self.serial_port.readline().decode().strip()
                    if response:
                        self.get_logger().info(f"STM32 response: {response}")
                    break
                except serial.SerialTimeoutException:
                    self.get_logger().warning(f"Serial timeout on attempt {attempt + 1}/{max_retries}")
                    if attempt == max_retries - 1:
                        self.get_logger().error("Serial write failed after max retries")
                        self._reset_serial()
                except serial.SerialException as e:
                    self.get_logger().error(f"Serial error: {e}")
                    self._reset_serial()
                    break
                time.sleep(0.1)

            # Serial health check
            if time.time() - self.last_serial_success > 2.0:
                self.get_logger().warning("No serial success for 2 seconds, attempting recovery")
                self._reset_serial()

        except Exception as e:
            self.get_logger().error(f"Error in control_loop: {e}")
            self._emergency_stop()

    def _reset_serial(self):
        try:
            if self.serial_port and self.serial_port.is_open:
                self.serial_port.close()
            self.serial_port = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
            self.serial_port.reset_input_buffer()
            self.serial_port.reset_output_buffer()
            self.last_serial_success = time.time()
            self.get_logger().info("Serial port recovered")
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to recover serial port: {e}")

    def _emergency_stop(self):
        try:
            if self.serial_port and self.serial_port.is_open:
                packet = struct.pack('<B4h4B', 0, 0, 0, 0, 0, 90, 90, 90, 90)
                self.serial_port.write(packet)
                self.get_logger().info("Emergency stop: Motors stopped, servos reset to (90, 90, 90, 90)")
        except Exception as e:
            self.get_logger().error(f"Error during emergency stop: {e}")

    def destroy_node(self):
        self._emergency_stop()
        try:
            if self.serial_port and self.serial_port.is_open:
                self.serial_port.close()
                self.get_logger().info("Serial port closed")
        except Exception as e:
            self.get_logger().error(f"Error closing serial port: {e}")
        self.udp_socket.close()
        self.get_logger().info("UDP socket closed")
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = RobotNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Node shutdown requested")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
