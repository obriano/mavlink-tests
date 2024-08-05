from pymavlink import mavutil
import time

# Connect to the Pixhawk
# Ensure the correct telemetry port and baud rate
master = mavutil.mavlink_connection('/dev/ttyAMA0', baud=57600)

# Wait for a heartbeat from the Pixhawk
print("Waiting for heartbeat...")
master.wait_heartbeat()
print("Heartbeat received")

# Arm the drone
print("Arming the drone...")
master.arducopter_arm()

# Confirm that the drone is armed
master.motors_armed_wait()
print("Drone is armed")

# Stay armed for 5 seconds
time.sleep(5)

# Disarm the drone
print("Disarming the drone...")
master.arducopter_disarm()

# Confirm that the drone is disarmed
master.motors_disarmed_wait()
print("Drone is disarmed")

print("Test complete")
