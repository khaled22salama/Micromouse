

## Autonomous Micromouse Project

### Overview

This repository contains the code and configurations for an autonomous Micromouse robot. The robot uses two wheels with motors, equipped with encoders, and is controlled using an Arduino Mega. The project includes PID control to manage motor speed and direction, enabling the robot to navigate autonomously in a maze.

### Table of Contents

- [Files Overview](#files-overview)
- [Hardware Requirements](#hardware-requirements)
- [Software Requirements](#software-requirements)
- [Installation and Setup](#installation-and-setup)
- [Usage](#usage)
- [PID Tuning](#pid-tuning)
- [Contributing](#contributing)
- [License](#license)

### Files Overview

- **main.ino**: The main code file that includes all the necessary functions to control the robot.
- **robot_config.h**: Contains pin definitions and configuration settings for the robot.
- **motorPID.h**: Defines PID control parameters and functions.
- **robotMoves.h**: Contains functions for various robot maneuvers (e.g., turning left, right, stopping).

### Hardware Requirements

- **Arduino Mega 2560**: Microcontroller to run the code.
- **Motors with Encoders**: Two DC motors with attached encoders for speed and distance measurement.
- **Motor Driver**: Motor driver compatible with the Arduino Mega.
- **Ultrasonic Sensors (optional)**: For detecting obstacles.
- **Magnetic Encoders**: To track the rotation of the motors.
- **Wiring and Connectors**: To connect all components.

### Software Requirements

- [Arduino IDE](https://www.arduino.cc/en/software) version 1.8.13 or later.
- Arduino PID Library (available in the Library Manager).
- Serial Plotter for debugging (optional).

### Installation and Setup

1. **Clone the Repository**:
   ```bash
   git clone https:https://github.com/khaled22salama/Micromouse.git
   ```
2. **Open the Project in Arduino IDE**:
   - Launch the Arduino IDE and open the `main.ino` file.
   
3. **Install Required Libraries**:
   - Go to `Sketch > Include Library > Manage Libraries`.
   - Install the **PID_v1** library.
   
4. **Configure the Robot**:
   - Modify `robot_config.h` to match your specific hardware setup (e.g., motor pins, encoder pins).

5. **Upload the Code**:
   - Connect the Arduino Mega to your computer.
   - Select the correct board and port under `Tools`.
   - Click on the `Upload` button to flash the code to the Arduino.

### Usage

- Power the robot with a suitable power source (e.g., battery).
- The robot will start executing the main loop and perform movements based on the PID control logic.
- Use the Serial Monitor (baud rate 9600) to debug or monitor performance.
  
### PID Tuning

PID (Proportional-Integral-Derivative) control is used to manage the speed of both motors. Tuning the PID parameters (`Kp`, `Ki`, and `Kd`) is crucial for optimal performance. 

- **`Kp` (Proportional Gain)**: Controls the response to the current error.
- **`Ki` (Integral Gain)**: Eliminates steady-state errors.
- **`Kd` (Derivative Gain)**: Reduces overshoot and dampens oscillations.

You can adjust these parameters in the `main.ino` file:
```cpp
double Kp1 = 0, Ki1 = 0, Kd1 = 0;  // Motor 1 PID parameters
double Kp2 = 0, Ki2 = 0, Kd2 = 0;  // Motor 2 PID parameters
```

### Contributing

Contributions are welcome! Please follow these steps:
1. Fork the repository.
2. Create a new branch (`git checkout -b feature-branch`).
3. Commit your changes (`git commit -am 'Add new feature'`).
4. Push to the branch (`git push origin feature-branch`).
5. Open a pull request.

### License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

