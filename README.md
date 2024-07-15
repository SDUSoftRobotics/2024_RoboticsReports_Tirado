# Feedback Control of a Modular Proprioceptive Soft Actuator

## Overview
This project involves the feedback control of a modular proprioceptive soft actuator using an Arduino for controlling the actuator and a Python script for data acquisition and sequence generation.

## Project Files

### Arduino Code
- **File**: `soft_actuator_control.ino`
- **Purpose**: This Arduino code is responsible for controlling the bend angle of a soft actuator. It uses a PID controller to adjust the actuator's angle based on sensor feedback.
- **Authors**: Jonathan Tirado, Joséphine Moisson de Vaux
- **Date**: May 13, 2023
- **Version**: 1.0

#### Key Components
- **Bendlabs Sensor**: Provides feedback on the bend angle of the actuator.
- **MPX5100 Series Integrated Silicon Pressure Sensor**: Measures pressure within the actuator.
- **PID Controller**: Maintains the desired angle by adjusting motor speed and valve states.

### Python Code
- **File**: `data_acquisition_and_sequence_generation.py`
- **Purpose**: This Python script handles data acquisition and sequence generation for the soft actuator. It communicates with the Arduino via serial to control the bend angle and logs sensor data to a CSV file.
- **Authors**: Jonathan Tirado, Joséphine Moisson de Vaux
- **Date**: May 15, 2023
- **Version**: 1.0

#### Key Functions
- **`bend_control(ser, bend_val)`**: Sends bend angle commands to the Arduino.
- **`main_loop(ser)`**: Controls the actuator's bend angle in a sequence and records sensor data.
- **`save_data(filename, data)`**: Saves the recorded data to a CSV file.

## Bend Sensor Information
The bend sensor used in this project is provided by Nitto, a leading manufacturer of bend sensors. For more information about the sensor, please visit [Nitto's website](https://www.nitto.com/jp/ja/nbt/).

## Getting Started

### Prerequisites
- **Arduino IDE**: To upload the Arduino code to your board.
- **Python**: To run the data acquisition and sequence generation script.
- **Serial Library**: Install using `pip install pyserial`.

### Setup
1. **Arduino Setup**:
    - **Libraries**:
        - Install the `ads` library for the Bendlabs sensor.
        - Install the `PID_v1` library for the PID controller.
    - Connect the Bendlabs sensor and MPX5100 pressure sensor to the Arduino as specified in the code.
    - Upload the `soft_actuator_control.ino` to your Arduino board.

2. **Python Setup**:
    - Connect your computer to the Arduino via a serial connection (e.g., COM7).
    - Run the `data_acquisition_and_sequence_generation.py` script to start data acquisition and control sequence.

### Running the Project
1. **Arduino**:
    - Ensure your sensors and actuators are properly connected.
    - Open the Arduino IDE, upload the code, and open the Serial Monitor to verify the setup.

2. **Python**:
    - Open a terminal or command prompt.
    - Navigate to the directory containing the Python script.
    - Run the script using `python data_acquisition_and_sequence_generation.py`.

### Data Logging
- The Python script logs sensor data to a CSV file named `linear_control_experiment_1.csv`.
- The data includes timestamps and sensor readings, which can be analyzed for further research and development.




