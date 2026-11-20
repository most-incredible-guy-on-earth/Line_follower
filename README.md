# Line_follower
This is my line follower robot project

# ESP32-Based PID Line Follower Robot

An autonomous line-following robot built using an ESP32 microcontroller, four analog IR sensors and a TB6612FNG motor driver. The robot uses sensor calibration, weighted-average line position estimation and PID (Proportional–Integral–Derivative) control to follow a track with differential motor-speed adjustment.

This project explores the fundamentals of embedded systems, sensor processing, feedback control systems and autonomous robotics.

---

## Project Demonstration

**Demo Video:** 

[▶ Watch the Robot in Action](YOUR_DEMONSTRATION_VIDEO_LINK_HERE)

> Replace the link above with your YouTube or Google Drive demonstration video.

**Project Images:**

Add photographs of the completed robot, sensor arrangement and electronics here.

---

## 1. Project Overview

The objective of this project was to design and build an autonomous mobile robot capable of following a predefined line on a surface.

The robot uses four analog infrared (IR) sensors to detect the position of the line. An ESP32 processes the sensor readings, estimates the line's position and calculates the deviation from the center.

A PID controller uses this error to dynamically adjust the speeds of the two DC motors, allowing the robot to correct its trajectory while moving.

Unlike a basic line follower that relies on simple left/right switching logic, this implementation uses continuous sensor readings and feedback-based motor control.

### Key Features

- Four-channel analog IR sensor array.
- Automatic sensor calibration.
- Sensor reading normalization.
- Weighted-average line position estimation.
- PID-based feedback control.
- Differential motor-speed adjustment.
- Line-loss detection and motor stopping.
- ESP32-based embedded control.
- TB6612FNG dual motor driver interface.

---

## 2. Hardware Components

| Component | Purpose |
|---|---|
| ESP32 Development Board | Main microcontroller |
| 4 Analog IR Sensors | Line detection |
| TB6612FNG | Dual DC motor driver |
| 2 DC Gear Motors | Robot locomotion |
| Robot Chassis | Mechanical structure |
| 4 × 1.5V AA Batteries | Motor power supply |
| Jumper Wires | Electrical connections |

---

## 3. Pin Configuration

### ESP32 Connections

| ESP32 GPIO | Connected Component | Function |
|---|---|---|
| GPIO 39 | IR Sensor 1 | Analog input |
| GPIO 34 | IR Sensor 2 | Analog input |
| GPIO 35 | IR Sensor 3 | Analog input |
| GPIO 32 | IR Sensor 4 | Analog input |
| GPIO 23 | Sensor Power Control | Sensor power |
| GPIO 17 | TB6612FNG PWMA | Motor PWM |
| GPIO 18 | TB6612FNG PWMB | Motor PWM |
| GPIO 16 | VCC variable in code | Driver control/power connection |

**Motor mapping in the code:**

- PWMA controls the right motor.
- PWMB controls the left motor.

**Note:** The direction inputs and standby connection of the TB6612FNG are not controlled by this final program. They must be configured through the actual hardware wiring.

GPIO 16 is configured as an output and switched LOW during calibration and HIGH afterward. If this connection is intended for the TB6612FNG logic VCC pin, connect VCC directly to a suitable regulated 3.3V supply instead. A GPIO must not be used to power the driver. Use the STBY pin for driver enable control if that is the intended function.

The motor supply VM and ESP32 ground must be wired appropriately, with a common ground.

---

## 4. Working Mechanism

The robot operates through a continuous feedback-control loop.

Its operation can be divided into the following stages:

### 4.1 Sensor Reading

Four analog IR sensors are positioned underneath the robot.

Each sensor measures the reflected infrared light from the surface.

In this implementation:

- White surface produces higher analog readings.
- Black line produces lower analog readings.

The ESP32 reads all four sensors using `analogRead()`.

The readings are stored in an array:

```cpp
int Sensors[4] = {39, 34, 35, 32};
int Readings[4] = {};
```

### 4.2 Sensor Calibration

Before the robot starts moving, it performs a 10-second calibration.

During calibration, the robot records the maximum and minimum readings observed by each sensor.

- Maximum reading represents the white surface.
- Minimum reading represents the black line.

This allows the robot to adapt to variations in sensor response and surface conditions.

The calibration process uses:

```cpp
white[j] = max(white[j], Readings[j]);
black[j] = min(black[j], Readings[j]);
```

The calibration assumes that every sensor encounters both black and white surfaces during the calibration period.

### 4.3 Sensor Normalization

Raw analog readings are converted into normalized strength values between 0 and 1000.

The normalization formula is:

\[
S_i =
\frac{W_i-R_i}{W_i-B_i}\times1000
\]

Where:

- \(S_i\) = Normalized sensor strength.
- \(W_i\) = Calibrated white reading.
- \(B_i\) = Calibrated black reading.
- \(R_i\) = Current sensor reading.

The resulting values are constrained between 0 and 1000.

Therefore:

- White surface → approximately 0.
- Black line → approximately 1000.

This makes sensor readings easier to process independently of their original analog ranges.

### 4.4 Weighted-Average Line Position

The robot estimates the position of the line using a weighted-average algorithm.

Each sensor is assigned a position weight:

```cpp
int weight[4] = {0, 1000, 2000, 3000};
int center = 1500;
```

The line position is calculated using:

\[
Position =
\frac{\sum_{i=0}^{3} S_iW_i}
{\sum_{i=0}^{3}S_i}
\]

Where:

- \(S_i\) represents the normalized sensor strength.
- \(W_i\) represents the corresponding sensor position weight.

The calculated position ranges approximately from 0 to 3000.

The center of the sensor array is defined as 1500.

If the total sensor strength is zero, the algorithm identifies that the line has been lost and returns -1.

### 4.5 Error Calculation

The deviation of the line from the center is calculated as:

\[
Error = Position - Center
\]

This establishes the following sign convention:

- Positive error → Line is towards the right.
- Negative error → Line is towards the left.
- Zero error → Line is centered.

The error becomes the input to the PID controller.

### 4.6 PID Controller

The robot uses a PID controller to calculate the required motor-speed correction.

The controller consists of three components:

**Proportional (P)**

Responds to the current error.

\[
P=K_p \times Error
\]

**Integral (I)**

Accumulates error over time to compensate for persistent deviations.

\[
I=I+K_i \times Error \times dt
\]

**Derivative (D)**

Measures the rate of change of error and helps reduce overshooting.

\[
D=K_d \times
\frac{Error-PreviousError}{dt}
\]

The final correction is:

\[
Correction=P+I+D
\]

The correction is constrained between -127.5 and +127.5 to limit the motor-speed adjustment.

### PID Parameters

The current implementation uses:

| Parameter | Value |
|---|---:|
| Kp | 0.08 |
| Ki | 0.0008 |
| Kd | 0.08 |
| Base Speed | 180 |
| Minimum Speed | 30 |
| Sampling Interval | 10 ms |

These are the working configuration values used in the project, not universal optimal PID values.

### 4.7 Differential Motor-Speed Adjustment

The robot uses differential steering.

Instead of directly controlling the steering angle, it adjusts the relative speeds of the left and right motors.

The motor speeds are calculated as:

\[
LeftSpeed=BaseSpeed+Correction
\]

\[
RightSpeed=BaseSpeed-Correction
\]

The base speed is also adjusted to maintain headroom for the correction:

\[
BaseSpeed=\min(180,255-|Correction|)
\]

Both motor speeds are constrained between 30 and 255 during normal line tracking.

This allows the robot to turn by changing the relative speeds of its two motors while maintaining forward motion.

### 4.8 Line-Loss Detection

When all four sensors detect no significant line strength, the position calculation returns -1.

The robot then:

- Sets both motor speeds to zero.
- Resets the integral term.
- Resets the previous error.
- Clears the derivative term.
- Activates the line-loss flag.

When the line is detected again, the previous error is synchronized with the newly calculated error to avoid an unwanted derivative spike during recovery.

---

## 5. Software Architecture

The program is divided into modular functions, each responsible for a specific operation.

| Function | Responsibility |
|---|---|
| `collect_readings()` | Reads analog values from all four sensors |
| `calibrate()` | Determines the black and white reference values |
| `calculate_strength()` | Normalizes raw sensor readings |
| `calculate_position()` | Calculates the weighted-average line position |
| `calculate_error()` | Determines deviation from the center |
| `PID()` | Calculates the proportional, integral and derivative corrections |
| `speed_adjustment()` | Converts PID correction into motor speeds |
| `motor_control()` | Sends PWM signals to the motor driver |

### Main Control Flow

```text
START
  |
  v
Initialize ESP32 and peripherals
  |
  v
Calibrate IR sensors
  |
  v
Read sensor values
  |
  v
Normalize sensor readings
  |
  v
Calculate line position
  |
  v
Is the line detected?
  |
  +---- NO ----> Stop both motors
  |                 |
  |                 v
  |             Reset PID state
  |
  +---- YES ---> Calculate error
                    |
                    v
                PID calculation
                    |
                    v
              Calculate motor speeds
                    |
                    v
                Apply PWM
                    |
                    v
               Repeat loop
```

---

## 6. Technologies and Techniques Used

**Hardware and Embedded Systems**
- ESP32 microcontroller.
- Analog infrared sensing.
- PWM motor-speed control.
- TB6612FNG motor driver.
- DC motor interfacing.

**Control Systems**
- Sensor calibration.
- Signal normalization.
- Weighted-average position estimation.
- PID feedback control.
- Differential steering.
- Line-loss detection.

**Programming**
- C++.
- Arduino framework.
- Modular programming.
- Real-time sensor processing.

---

## 7. Setup and Usage

### Step 1: Assemble the Hardware

Connect the ESP32, four analog IR sensors, TB6612FNG, two DC motors and battery pack according to the pin configuration and your actual wiring.

Ensure that:
- The sensors are arranged from left to right in the expected array order.
- The motor directions are configured correctly.
- The driver has a suitable logic supply and motor supply.
- All required grounds are connected together.

### Step 2: Configure the Arduino IDE

Install the ESP32 board package and select the appropriate ESP32 development board.

### Step 3: Upload the Program

Copy the project source code into the Arduino IDE and upload it to the ESP32.

### Step 4: Perform Calibration

Place the robot over the track and ensure that all four sensors encounter both black and white surfaces during the 10-second calibration period.

### Step 5: Run the Robot

After calibration, the program continuously reads sensor values, calculates the line position, processes PID corrections and adjusts the motor speeds.

---

## 8. Challenges and Learning Outcomes

This project involved understanding and implementing several practical robotics concepts.

Some of the major learning areas included:

- Understanding the relationship between analog sensor readings and surface reflectivity.
- Designing a calibration process for multiple sensors.
- Converting raw sensor readings into meaningful position data.
- Understanding weighted-average algorithms for position estimation.
- Implementing a PID controller from scratch.
- Converting PID output into differential motor speeds.
- Interfacing an ESP32 with a dual motor driver.
- Debugging hardware connections and motor-control issues.
- Understanding feedback-based autonomous movement.

The project provided practical experience in combining hardware, software and control theory to build an autonomous robotic system.

---

## 9. Future Improvements

Potential improvements for future versions include:

- Increasing the number of IR sensors for better line-position resolution.
- Implementing adaptive PID tuning.
- Adding encoder feedback for closed-loop wheel-speed control.
- Implementing more advanced line-loss recovery.
- Improving speed optimization on sharp turns.
- Adding wireless telemetry for real-time monitoring.
- Developing a more robust calibration and sensor-noise filtering system.

---

## 10. Project Status

**Version:** 1.0

**Status:** Completed – Working Prototype

This project represents my first practical implementation of a PID-controlled autonomous mobile robot.

It is also a foundation for exploring more advanced robotics concepts, including ROS 2, autonomous navigation, sensor fusion and intelligent robotic systems.

---

## 11. Author

**Developed by:** [Your Name]

**GitHub:** [Your GitHub Profile]

**LinkedIn:** [Your LinkedIn Profile]

---

*Built to explore the fundamentals of autonomous robotics, embedded systems and feedback control.*

