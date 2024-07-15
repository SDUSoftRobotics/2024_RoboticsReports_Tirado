# -*- coding: utf-8 -*-
"""
Created on Mon May 15 14:17:15 2023

@author: SDU
"""

import serial
import time
import csv

# Initialize serial connection
ser = serial.Serial("COM7", 115200, timeout=1)

# Initialize variables
data = []
angle = -10
direction = 1
start_time = time.time()
line_count = 0

def bend_control(ser, bend_val):
    """
    Control the bend angle of the actuator.

    Parameters:
    ser (serial.Serial): Serial connection object.
    bend_val (int): Bend angle value in the range of 0 to +180.
    """
    ser_bytes = ser.readline()
    ser.flushInput()
    time.sleep(0.001)
    motor_bend = str(bend_val)
    cmd = "a" + motor_bend + "\n"
    ser.write(bytes(cmd, encoding="ascii"))

def main_loop(ser):
    """
    Main loop to control the bend and record data.
    
    Parameters:
    ser (serial.Serial): Serial connection object.
    """
    global angle, direction, line_count, start_time
    while time.time() - start_time < 1050:  # Loop until total record time has elapsed
        record_time = time.time()
        angle += direction * 10
        if angle > 100 or angle < 0:
            direction *= -1
        bend_control(ser, angle)
    
        while time.time() - record_time <= 50:  # Record data for 50 seconds
            line = ser.readline().decode("utf-8").rstrip()
            line_count += 1  # Increment line counter
    
            # Ignore initial motion setup lines
            if line_count <= 1:
                print("Omitting initial setup line:", line)
                continue
    
            # Split data into list of values using comma as delimiter
            values = line.split(',')
            print(values)
    
            if len(values) < 4:
                print("Omitting incomplete line:", line)
                continue
    
            # Convert values to float and append to data list
            try:
                data.append([time.time() - start_time, values[0], values[1], values[2], values[3]])
            except ValueError:
                continue
            
            time.sleep(0.01)

def save_data(filename, data):
    """
    Save the recorded data to a CSV file.

    Parameters:
    filename (str): The name of the file to save the data.
    data (list): The data to be saved.
    """
    with open(filename, "w", newline="") as file:
        writer = csv.writer(file)
        writer.writerow(["Time (s)", "Value 1", "Value 2", "Value 3", "Value 4"])
        writer.writerows(data)
    print("Data saved to file:", filename)

if __name__ == "__main__":
    try:
        main_loop(ser)
    finally:
        ser.close()
        save_data("linear_control_experiment_1.csv", data)
