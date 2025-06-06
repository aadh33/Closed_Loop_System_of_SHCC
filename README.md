Closed Loop System of Ship Hull Cleaning Crawler (SHCC)
Welcome to the Closed Loop System of Ship Hull Cleaning Crawler (SHCC) repository. This project was developed as a critical part of the embedded systems internship focused on automating and optimizing underwater hull cleaning operations using a crawler robot. The system implements a robust closed-loop control architecture to precisely navigate, control, and clean ship hulls efficiently and safely.

📖 Project Overview
The SHCC Closed Loop System is designed to control a robotic crawler that cleans the underwater hull of ships autonomously or via manual joystick control. The system uses sensor feedback to continuously monitor and adjust motor speeds and directions, ensuring accurate motion control and effective cleaning operations.

This repository contains the firmware, algorithms, and control logic developed during the internship, including embedded firmware for microcontrollers, communication protocols, sensor integration, and control system architecture.

🚢 Background & Motivation
Hull cleaning is an essential maintenance process for ships to remove biofouling such as algae, barnacles, and other marine organisms. Manual cleaning is time-consuming, labor-intensive, and often unsafe due to underwater hazards.

Automating this process using an intelligent crawler robot with a closed-loop control system dramatically improves:

Safety by reducing human underwater intervention

Efficiency with precise movement and coverage of hull surfaces

Repeatability ensuring consistent cleaning performance

Cost-effectiveness by reducing operational downtime and labor costs

🔧 System Architecture
1. Hardware Components
ESP32S3 Microcontroller
Acts as the main processing unit for sensor integration, motor control, and communication.

Arduino Leonardo
Handles encoder data acquisition and preprocessing for precise motion feedback.

Motor Drivers & Motors
Control the propulsion and cleaning mechanisms of the crawler.

Rotary Encoders
Provide real-time rotational position and speed feedback from the motors.

Joystick Interface
Enables manual control via ROS2 publisher or WebSocket input.

Sensors
Include distance sensors, IMU, and environmental sensors for enhanced control (optional).

2. Software Components
Closed Loop Control Algorithm
Implements PID or other feedback control strategies using encoder feedback to maintain desired speed and position.

Communication Protocols
UART between Arduino Leonardo and ESP32S3 for encoder data.
Ethernet communication from Python ROS2 publisher for joystick commands.

ROS2 Integration
Allows high-level command and control, bridging manual inputs and automated routines.

WebSocket Server in Python
Facilitates remote joystick control from a web interface or external device.

⚙️ Key Features
Real-Time Feedback Control
Motor speeds are continuously adjusted based on encoder feedback to ensure accurate trajectory and cleaning patterns.

Multi-Device Communication
Seamless integration between microcontrollers and external control interfaces.

Modular Firmware Design
Easily extensible codebase for adding new sensors, control strategies, or cleaning mechanisms.

Safety Mechanisms
Emergency stop and fault detection for preventing damage or unsafe operation.

Joystick Control via ROS2
Real-time manual override and remote operation capability.

🛠️ Installation & Setup
Hardware Assembly
Follow the wiring schematics provided in the docs/ folder for microcontroller and sensor connections.

Firmware Upload

Flash the ESP32S3 firmware via USB using ESP-IDF or Arduino IDE.

Upload Arduino Leonardo sketch for encoder data acquisition.

Software Dependencies

Install ROS2 Foxy or later.

Install Python dependencies for WebSocket and ROS2 communication:
pip install websockets rclpy

Running the System-
Start the ROS2 joystick publisher node.
Launch the WebSocket server for remote control.
Power the robot and initiate closed-loop motor control.

📝 Code Structure
/SHCC_Closed_Loop_System
│
├── firmware/
│   ├── esp32s3_control/        # ESP32S3 motor control and communication code
│   ├── arduino_leonardo/       # Encoder reading and data transmission
│
├── ros2_nodes/
│   ├── joystick_publisher.py  # ROS2 node publishing joystick commands
│
├── websocket_server/
│   ├── ws_server.py            # WebSocket server for joystick interface
│
├── docs/
│   ├── wiring_diagram.png
│   ├── system_architecture.pdf
│
├── README.md                  

📊 Performance & Results
Achieved precise velocity control with encoder feedback under various load conditions.

Smooth joystick-based manual control with negligible latency through ROS2 and WebSocket integration.

Robust error handling to maintain operational safety during real-world testing.

Modular design allows easy scaling for multi-motor and multi-sensor setups.

🚀 Future Work
Implement autonomous path planning using onboard sensors and SLAM techniques.

Add more environmental sensors for adaptive cleaning strategies.

Integrate machine learning models for anomaly detection and predictive maintenance.

Expand WebSocket interface to mobile platforms for enhanced remote usability.

🙌 Acknowledgments
This project was developed during an embedded systems internship at [Company Name], with invaluable guidance from mentors and the robotics team. Special thanks to the firmware and testing teams for their collaborative efforts.

📄 License
This project is licensed under the MIT License - see the LICENSE file for details.

📬 Contact
For questions, suggestions, or collaboration opportunities, please reach out:

Author: Adithyan Manoj

Email: [adithyanmanoj33@example.com]

LinkedIn: https://linkedin.com/in/adithyan-manoj

GitHub: https://github.com/aadh33

Thank you for visiting the SHCC Closed Loop System repository!
We hope this project inspires further innovation in underwater robotics and embedded control systems.
