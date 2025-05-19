from XRPLib.differential_drive import DifferentialDrive
from machine import Timer
import time, math, gc, os

drivetrain = DifferentialDrive.get_default_differential_drive()
gc.collect()  # empty RAM

# Constants
counts_per_wheel_revolution = (30/14) * (28/16) * (36/9) * (26/8) * 12  # 585
wheel_diameter = 60  # mm
circumference_wheel = math.pi * wheel_diameter  # 188.49 mm
sampling_time = 0.2  # s
hardware_timer_period = sampling_time

# Data collection
data = []  # temp. storage
data_interval = 20  # ms
filename = "data.csv"

if filename in os.listdir():
    os.remove(filename)
    print(f"{filename} deleted.")
else:
    print(f"{filename} does not exist.")

class PositionEstimation:
    def __init__(self):
        self.track_width = 15.5  # cm
        self.wheel_diam = 6  # cm
        self.RPMtoCMPS = (math.pi * self.wheel_diam) / 60  # cm/s per RPM
        self.x = 0
        self.y = 0
        self.theta = 0
        self.last_data_time = time.ticks_ms()

    def pose_update(self, t=None):
        right_motor_speed = drivetrain.right_motor.get_speed() * self.RPMtoCMPS
        left_motor_speed = drivetrain.left_motor.get_speed() * self.RPMtoCMPS

        if abs(right_motor_speed - left_motor_speed) < 1e-6:
            # Straight line motion
            v = right_motor_speed
            dx = v * math.cos(self.theta) * hardware_timer_period
            dy = v * math.sin(self.theta) * hardware_timer_period
            dtheta = 0
        else:
            # Turning motion
            R = 0.5 *self.track_width*(right_motor_speed + left_motor_speed) / (right_motor_speed - left_motor_speed)
            w = (right_motor_speed - left_motor_speed) / self.track_width
            wt = w * hardware_timer_period
            dx = -R * math.sin(self.theta) + R * math.sin(self.theta + wt)
            dy = R * math.cos(self.theta) - R * math.cos(self.theta + wt)
            dtheta = wt

        self.x += dx
        self.y += dy
        self.theta += dtheta

        # Store every data_interval
        current_time = time.ticks_ms()
        if time.ticks_diff(current_time, self.last_data_time) >= data_interval:
            data.append([self.x, self.y, self.theta])
            self.last_data_time = current_time

kinematics = PositionEstimation()

# Start pose update timer
timer = Timer()
DT = hardware_timer_period
timer.init(period=int(DT * 1000), mode=Timer.PERIODIC, callback=lambda t: kinematics.pose_update(t))

# Robot behavior
def execute_trajectory():
    velocity = 3
    turn_speed = 20

    motion_sequence = [
        (velocity, velocity, 10),  # Move forward
        (0, 0, 1),
        (0, 0, 1),
        (0, 0, 1),
    ]

    for left, right, duration in motion_sequence:
        drivetrain.set_speed(left, right)
        time.sleep(duration)

    drivetrain.turn(180)
    time.sleep(3)

    motion_sequence_continue = [
        (velocity, velocity, 10),
        (0, 0, 1)
    ]

    for left, right, duration in motion_sequence_continue:
        drivetrain.set_speed(left, right)
        time.sleep(duration)

execute_trajectory()

# Stop the pose update timer
timer.deinit()

# Write data to CSV
with open(filename, "w") as f:
    f.write('x,y,theta\n')
    for row in data:
        f.write(f"{row[0]},{row[1]},{row[2]}\n")

# Print data to serial
print("PART 1 BEGIN")
print("x,y,theta")
for row in data:
    print(f"{row[0]},{row[1]},{row[2]}")
print("PART 1 END")
print("experiment 1 completed")

