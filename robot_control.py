import serial
import time
import numpy as np

class RobotControl:
    def __init__(self, port='/dev/ttyACM0', baudrate=115200):
        """Initialize robot control interface"""
        self.serial = serial.Serial(port, baudrate)
        time.sleep(2)  # Wait for Arduino to reset
        
        # SBUS channel constants
        self.LOW = 192
        self.MID = 992
        self.HIGH = 1792
        self.dead_zone = 100
        
    def send_command(self, cmd):
        """Send a command string to the robot"""
        self.serial.write(f"{cmd}\n".encode())
        time.sleep(0.02)  # Small delay to ensure command is processed
        
    def move(self, strafe, forward, turn, pitch=0):
        """Send movement command
        Args:
            strafe: Left/Right motion (-1 to 1)
            forward: Forward/Backward motion (-1 to 1)
            turn: Turning motion (-1 to 1)
            pitch: Pitch up/down motion (-1 to 1)
        """

        # Convert -1 to 1 range to SBUS values
        strafe_val = int(self.MID + (strafe * (self.HIGH - self.MID)))
        forward_val = int(self.MID + (forward * (self.HIGH - self.MID)))
        turn_val = int(self.MID + (turn * (self.HIGH - self.MID)))
        pitch_val = int(self.MID + (pitch * (self.HIGH - self.MID)))
        
        # Constrain values
        strafe_val = max(self.LOW, min(self.HIGH, strafe_val))
        forward_val = max(self.LOW, min(self.HIGH, forward_val))
        turn_val = max(self.LOW, min(self.HIGH, turn_val))
        pitch_val = max(self.LOW, min(self.HIGH, pitch_val))

        #print("forward: {} turn: {}".format(forward_val, turn_val))
        
        self.send_command(f"move:{strafe_val},{forward_val},{turn_val},{pitch_val}")
        
    def unlock(self):
        """Unlock the robot (Mode 1, Channel 7)"""
        self.send_command("unlock")
        
    def toggle_damping(self):
        """Toggle damping mode (Mode 1, Channel 6)"""
        self.send_command("damping")
        
    def recover(self):
        """Execute fall recovery (Mode 2, Channel 7)"""
        self.send_command("recover")
        
    def toggle_continuous_movement(self):
        """Toggle continuous movement (Mode 2, Channel 6)"""
        self.send_command("continuous")
        
    def toggle_stand_height(self):
        """Toggle between high and low stand heights (Mode 1, Channel 8)"""
        self.send_command("stand")
        
    def toggle_obstacle_avoidance(self):
        """Toggle obstacle avoidance mode (Mode 2, Channel 8)"""
        self.send_command("obstacle")
        
    def switch_gait(self):
        """Switch walking gait (Mode 3, Channel 8 high)"""
        self.send_command("gait")
        
    def toggle_light(self):
        """Toggle LED light (Mode 3, Channel 8 low)"""
        self.send_command("light")
        
    def dance(self):
        """Execute dance sequence (Mode 3, Channel 7)"""
        self.send_command("dance")
        
    def jump(self):
        """Execute jump sequence (Mode 3, Channel 6)"""
        self.send_command("jump")
        
    def stop(self):
        """Stop all motion"""
        self.move(0, 0, 0, 0)
        
    def close(self):
        """Close the serial connection"""
        self.stop()
        self.serial.close()

def main():
    """Example usage of the robot control interface"""
    try:
        # Initialize robot control
        robot = RobotControl()
        
        # Unlock the robot
        print("Unlocking robot...")
        robot.unlock()
        time.sleep(1)
        
        # Move forward
        # print("Moving forward...")
        # robot.move(0, 0.0, 0.5)  # 50% forward speed
        # time.sleep(3)
        
        # # Stop
        # print("Stopping...")
        # robot.stop()
        # time.sleep(1)
        
        # # Execute some special commands
        # print("Performing dance...")
        # robot.dance()
        # time.sleep(3)

        # # Switch gait
        # print("Switching gait...")
        # robot.switch_gait()
        # time.sleep(2)

        # toggle light
        # print("Toggling light...")
        # robot.toggle_light()
        # time.sleep(1)

        # # toggle continuous movement
        # print("Toggling continuous movement...")
        # robot.toggle_continuous_movement()
        # time.sleep(1)

        # Move forward
        print("Moving forward...")
        start_time = time.time()
        while time.time() - start_time < 2:
            robot.move(0, -0.5, 0.5)  # 50% forward speed
            time.sleep(0.05)
        
        # print("Jumping...")
        # robot.jump()
        # time.sleep(2)
        
        # print("Toggling stand height...")
        # robot.toggle_stand_height()
        # time.sleep(2)

        # print("Damping mode...")
        # robot.toggle_damping()
        # time.sleep(1)
        
    except KeyboardInterrupt:
        print("\nStopping robot...")
    finally:
        robot.close()

if __name__ == "__main__":
    main()