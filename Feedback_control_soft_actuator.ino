/*
 * Project: Feedback Control of a Modular Proprioceptive Soft Actuator
 * Authors: Joséphine Moisson de Vaux, Jonathan Tirado
 * Date: May 14, 2023
 * Version: 1.0
 */

#include "Arduino.h"
#include <PID_v1.h>
#include "ads.h"

// Define the input pins for the sensors
#define ADS_RESET_PIN      (18)  // Bendlabs sensor Pin number attached to ADS reset line
#define ADS_INTERRUPT_PIN  (19)  // Bendlabs sensor Pin number attached to ADS interrupt line
#define PRESSURE_SENSOR    (A1)  // MPX5100 Series Integrated Silicon Pressure Sensor analog input (0 to 100 kPa)

// Arduino PWM Speed Control pins
int E1 = 3;  // Motor1 Speed
int M1 = 4;  // Motor1 Direction
int E2 = 11; // Motor2 Speed
int M2 = 12; // Motor2 Direction

const int E3 = 5;  // Motor3 Speed
const int E4 = 6;  // Motor4 Speed
const int M3 = 8;  // Motor3 Direction
const int M4 = 7;  // Motor4 Direction

// PID control parameters for angle control
int Kp_a = 5;
int Ki_a = 1;
int Kd_a = 0.5;

double Setpoint_a = 10; // Desired angle value (hardcoded)
double Input_a;  // Current angle value
double Output_a; // PID output

float p_limit = 30; // Pressure limit

// Variables for sensor values and commands
float pressure_sensorValue;
float angle;
int Output;
int Output2;

char command, commanda;
int incominglen = 0;
float value_f, value_f_a;

// PID controller object for angle control
PID angle_PID(&Input_a, &Output_a, &Setpoint_a, Kp_a, Ki_a, Kd_a, DIRECT);

// Function declarations for Bendlabs sensor data processing
void ads_data_callback(float *sample);
void deadzone_filter(float *sample);
void signal_filter(float *sample);
void parse_com_port(void);

/* Not used in polled mode. Stub function necessary for library compilation */
void ads_data_callback(float * sample, uint8_t sample_type)
{
}

// Motor control function declarations
void motor_1_on(int motorspeed);
void motor_1_off(void);
void valve_1_on(void);
void valve_1_off(void);
void valve_2_on(void);
void valve_2_off(void);

void setup() {
  // Setup motor control pins as outputs
  pinMode(M1, OUTPUT);
  pinMode(M2, OUTPUT);
  pinMode(M4, OUTPUT);
  pinMode(PRESSURE_SENSOR, INPUT);
  
  Serial.begin(115200); // Initialize serial communication
  
  // Turn the PID on and set initial PID values
  angle_PID.SetMode(AUTOMATIC);
  angle_PID.SetTunings(Kp_a, Ki_a, Kd_a);
  angle_PID.SetOutputLimits(-255, 255);
  
  Serial.println("Initializing One Axis sensor");
  
  // Initialize Bendlabs sensor
  ads_init_t init;
  init.sps = ADS_100_HZ; // Set sample rate to 100 Hz (Interrupt mode)
  init.ads_sample_callback = &ads_data_callback; // Provide callback for new data
  init.reset_pin = ADS_RESET_PIN; // Pin connected to ADS reset line
  init.datardy_pin = ADS_INTERRUPT_PIN; // Pin connected to ADS data ready interrupt
  init.addr = 0; // Update value if non-default I2C address is assigned to sensor
  
  int ret_val = ads_init(&init); // Initialize ADS hardware abstraction layer
  if (ret_val != ADS_OK) {
    Serial.print("One Axis ADS initialization failed with reason: ");
    Serial.println(ret_val);
  } else {
    Serial.println("One Axis ADS initialization succeeded...");
  }
  
  // Enable stretch measurements and start reading data in polled mode
  ads_stretch_en(true);
  ads_polled(true);
  
  // Wait for the first sample
  delay(10);
}

void loop() {
  float Setpoint_f;
  
  // Read angle sensor data
  float sample[2]; // Declare 'sample' as an array of floats
  uint8_t data_type; // Declare 'data_type' as an integer
  
  // Read data from the one axis ADS sensor
  int ret_val = ads_read_polled(sample, &data_type);
  if (ret_val == ADS_OK) {
    if (data_type == ADS_SAMPLE) {
      // Apply signal and deadzone filters to the sensor data
      signal_filter(sample);
      deadzone_filter(sample);
    }
  }
  
  // Define linear equation parameters
  float a = 1.15;
  float b = -5.18;
  
  // Calculate the angle from sensor data
  angle = a * sample[0] + b;

  // Check for received commands on the serial port
  if (Serial.available()) {
    commanda = command;
    value_f_a = value_f;
    String incomingValue = Serial.readStringUntil('\n');
    command = incomingValue[0];
    incominglen = incomingValue.length();
    String value = incomingValue.substring(1, incominglen);
    value_f = value.toFloat();
    
    if (command == 'c') {
      char calibrator = incomingValue[1];
      parse_com_port(calibrator);
    }
  }

  // Update setpoints and outputs based on received commands
  if (command == 'a') {
    Setpoint_a = value_f;
    Setpoint_f = Setpoint_a;
    Output = Output_a;
  } else if (command == 'm') {
    Setpoint_f = 0;
    Output = value_f;
  } else if (command == 'r') {
    Serial.print(sample[0]); // Angle data
    Serial.print(",");
    Serial.print(a * sample[0] + b); // Linearized angle data
    Serial.print(",");
    Serial.print(value_f);
    Serial.print(",");
    Serial.print(Output);
    Serial.println(",");
    command = commanda;
    Setpoint_a = value_f;
    Setpoint_f = Setpoint_a;
    Output = Output_a;
  }

  // Set the angle input value
  Input_a = angle;

  // Compute the PID output
  angle_PID.Compute();

  // Apply the PID output to the motor
  int motorSpeed = map(Output_a, -255, 255, 0, 255);
  if (command == 'a') {
    Output = Output_a;
    p_limit = 30;
  } else if (command == 'm') {
    Setpoint_f = 0;
    Output = value_f;
    p_limit = 27;
  }

  // Control motor and valves based on pressure sensor value and angle
  if (pressure_sensorValue <= p_limit && angle <= 185) {
    if (Output >= 0) {
      Output2 = map(Output, 0, 255, 15, 250);
      motor_1_on(Output2);
      valve_2_on();
      int error = Input_a - Setpoint_a;
      if (error < 0 && Output2 >= 100) {
        valve_2_off();
        delay(0.5);
        valve_2_on();
      } else {
        valve_2_on();
      }
    } else {
      motor_1_off();
      valve_2_off();
    }
  } else {
    motor_1_off();
    valve_2_off();
  }

  // Print debug information
  Serial.print(sample[0]);
  Serial.print(",");
  Serial.print(angle);
  Serial.print(",");
  Serial.print(Setpoint_a);
  Serial.print(",");
  Serial.println(Output_a);

  delay(10);
}

double getAngle() {
  // Implement the code to read the angle sensor and return the angle value
  return 0.0;
}

void parse_com_port(char key) {
  // Parse received characters from the COM port for commands
  switch (key) {
    case '0':
      ads_calibrate(ADS_CALIBRATE_FIRST, 0);
      Serial.println("0 calibrated");
      break;
    case '9':
      ads_calibrate(ADS_CALIBRATE_SECOND, 90);
      Serial.println("90 calibrated");
      break;
    case 'c':
      ads_calibrate(ADS_CALIBRATE_CLEAR, 0);
      break;
    case 'r':
      ads_run(true);
      break;
    case 's':
      ads_run(false);
      break;
    case 'f':
      ads_set_sample_rate(ADS_200_HZ);
      break;
    case 'u':
      ads_set_sample_rate(ADS_10_HZ);
      break;
    case 'n':
      ads_set_sample_rate(ADS_100_HZ);
      break;
    case 'b':
      ads_calibrate(ADS_CALIBRATE_STRETCH_ZERO, 0);
      break;
    case 'e':
      ads_calibrate(ADS_CALIBRATE_STRETCH_SECOND, 30);
      break;
    default:
      break;
  }
}

void signal_filter(float *sample) {
  // Define the coefficients
  const float c_a = 0.36952737735124147f;
  const float c_b = -0.19581571265583314f;
  const float c_c = 0.20657208382614792f;
  const float c_d = 2.0f;
  const float c_e = 1.0f;

  // Second order Infinite impulse response low pass filter (20 Hz cutoff frequency @ 100 Hz Sample Rate)
  static float filter_samples[2][6];
  for (uint8_t i = 0; i < 2; i++) {
    filter_samples[i][5] = filter_samples[i][4];
    filter_samples[i][4] = filter_samples[i][3];
    filter_samples[i][3] = (float)sample[i];
    filter_samples[i][2] = filter_samples[i][1];
    filter_samples[i][1] = filter_samples[i][0];

    // Apply the filter using the coefficients
    filter_samples[i][0] = filter_samples[i][1] * c_a +
                           filter_samples[i][2] * c_b +
                           c_c * (filter_samples[i][3] + c_d * filter_samples[i][4] + filter_samples[i][5]);

    sample[i] = filter_samples[i][0];
  }
}

void deadzone_filter(float *sample) {
  // Remove jitter from the signal by applying a deadzone filter
  static float prev_sample[2];
  float dead_zone = 0.75f;
  for (uint8_t i = 0; i < 2; i++) {
    if (fabs(sample[i] - prev_sample[i]) > dead_zone) {
      prev_sample[i] = sample[i];
    } else {
      sample[i] = prev_sample[i];
    }
  }
}

void motor_1_on(int motorspeed) {
  // Turn on motor 1 with the specified speed
  analogWrite(E1, motorspeed); // PWM Speed Control value
  digitalWrite(M1, HIGH);
}

void motor_1_off(void) {
  // Turn off motor 1
  analogWrite(E1, 0); // PWM Speed Control value
  digitalWrite(M1, HIGH);
}

void valve_1_on(void) {
  // Activate valve 1
  analogWrite(E2, 255); // PWM Speed Control value
  digitalWrite(M2, HIGH);
}

void valve_1_off(void) {
  // Deactivate valve 1
  analogWrite(E2, 0); // PWM Speed Control value
  digitalWrite(M2, HIGH);
}

void valve_2_on(void) {
  // Activate valve 2
  analogWrite(E4, 255); // PWM Speed Control value
  digitalWrite(M4, HIGH);
  analogWrite(E2, 255); // PWM Speed Control value
  digitalWrite(M2, HIGH);
}

void valve_2_off(void) {
  // Deactivate valve 2
  analogWrite(E4, 0); // PWM Speed Control value
  digitalWrite(M4, HIGH);
  analogWrite(E2, 0); // PWM Speed Control value
  digitalWrite(M2, HIGH);
}
