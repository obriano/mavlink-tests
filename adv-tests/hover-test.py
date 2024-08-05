import time
import serial
from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil
import argparse
# Function to set up the vehicle to hover
def hover(vehicle):
    print("Setting vehicle to hover at current location")
    vehicle.mode = VehicleMode("GUIDED")
    location = vehicle.location.global_relative_frame
    vehicle.simple_goto(location)
# Function to send data via UART
def send_uart_data(serial_port, data):
    serial_port.write(data.encode())
# Connect to the vehicle
connection_string = '127.0.0.1:14550'
print('Connecting to vehicle on:', connection_string)
vehicle = connect(connection_string, wait_ready=True)
# Save the initial location
initial_location = vehicle.location.global_relative_frame
# Set up UART connection
#uart_port = '/dev/tty0'  # Replace with your actual serial port
# baud_rate = 9600  # Adjust to your baud rate
#try:
    #serial_port = serial.Serial(uart_port, baud_rate, timeout=1)
#except serial.SerialException as e:
    #print(f"Error opening serial port {uart_port}: {e}")
   # exit(1)
def arm_and_takeoff(target_altitude):
    """
    Arms vehicle and fly to target_altitude.
    """
    print("Basic pre-arm checks")
    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)
    print("Arming motors")
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True
    while not vehicle.armed:
        print(" Waiting for arming...")
        time.sleep(1)
    print("Taking off!")
    vehicle.simple_takeoff(target_altitude)
    while True:
        print(" Altitude: ", vehicle.location.global_relative_frame.alt)
        if vehicle.location.global_relative_frame.alt >= target_altitude * 0.95:
            print("Reached target altitude")
            break
        time.sleep(1)
# Arm and take off to 10 meters altitude
arm_and_takeoff(10)
# Hover at the current position
hover(vehicle)
# Example data to send via UART
data_to_send = "Hovering at 10 meters altitude"
# Send data via UART
#send_uart_data(serial_port, data_to_send)
# Keep the vehicle hovering and sending data
try:
    while True:
        # Maintain hover by sending the same location command periodically
        hover(vehicle)
        
        # Send data periodically
     #   send_uart_data(serial_port, data_to_send)
        
        time.sleep(1)
except KeyboardInterrupt:
    print("Exiting...")
# Function to return to the initial position and land the drone
def return_and_land(vehicle, initial_location):
    """
    Returns the vehicle to its initial location and lands it.
    
    Args:
    vehicle: Vehicle object
    initial_location: Initial location (LocationGlobalRelative object)
    """
    print("Returning to initial position")
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.simple_goto(initial_location)
    
    # Wait until the vehicle reaches the initial location
    while True:
        current_location = vehicle.location.global_relative_frame
        distance = get_distance_metres(current_location, initial_location)
        print(f"Distance to initial location: {distance:.2f} meters")
        
        if distance < 1.0:  # If within 1 meter of the target, proceed to land
            print("Reached initial location, initiating landing...")
            vehicle.mode = VehicleMode("LAND")
            break
        time.sleep(1)
    # Wait until the vehicle lands
    while vehicle.armed:
        print(" Waiting for landing to complete...")
        if vehicle.location.global_relative_frame.alt <= 0.1:
            print("Landed successfully.")
            vehicle.armed = False
            break
        time.sleep(1)
# Function to calculate distance between two locations
def get_distance_metres(aLocation1, aLocation2):
    """
    Returns the ground distance in metres between two LocationGlobalRelative objects.
    
    Args:
    aLocation1: LocationGlobalRelative object
    aLocation2: LocationGlobalRelative object
    
    Returns:
    Distance in meters.
    """
    dlat = aLocation2.lat - aLocation1.lat
    dlong = aLocation2.lon - aLocation1.lon
    return ((dlat**2) + (dlong**2))**0.5 * 1.113195e5
# Uncomment and set up UART connection if needed
# uart_port = '/dev/tty0'  # Replace with your actual serial port
# baud_rate = 9600  # Adjust to your baud rate
# try:
#     serial_port = serial.Serial(uart_port, baud_rate, timeout=1)
# except serial.SerialException as e:
#     print(f"Error opening serial port {uart_port}: {e}")
#     exit(1)
# Wait for the vehicle to be at a safe altitude to return and land
print("Waiting for vehicle to hover at a safe altitude...")
time.sleep(10)  # Adjust this sleep time based on your hover stability
# Return to the initial position and land
return_and_land(vehicle, initial_location)
# Close connection
print("Close vehicle object")
vehicle.close()
# if 'serial_port' in locals():
#     serial_port.close()

