from dronekit import connect, VehicleMode
import time

# Connect to the vehicle
vehicle = connect('127.0.0.1:14550', wait_ready=True)

# Arm the vehicle
vehicle.mode = VehicleMode("GUIDED")
vehicle.armed = True
while not vehicle.armed:
    print("Waiting for arming...")
    time.sleep(1)
print("Vehicle is armed")

# Stay armed for 5 seconds
time.sleep(5)

# Disarm the vehicle
vehicle.armed = False
while vehicle.armed:
    print("Waiting for disarming...")
    time.sleep(1)
print("Vehicle is disarmed")

# Close vehicle connection
vehicle.close()
