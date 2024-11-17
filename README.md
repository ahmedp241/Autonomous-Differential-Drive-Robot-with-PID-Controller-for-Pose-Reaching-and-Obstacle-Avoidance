# **Autonomous Differential Drive Robot with PID Controller for Pose Reaching and Obstacle Avoidance**  
**Using Arduino and MATLAB/Simulink Simulation**

---

## **Abstract**  
This project implements an autonomous differential-drive robot capable of navigating to a desired pose while avoiding obstacles in real time. The system integrates hardware control through Arduino and a simulation environment developed in MATLAB/Simulink for validation. It leverages sensor inputs for obstacle detection and calculates dynamic trajectories to achieve precise navigation. This hybrid approach ensures robust and efficient performance suitable for educational and practical robotics applications.

---

## **Problem Statement**  
The goal is to move a robot from its current pose `(x, y, θ)` to a desired pose `(x_d, y_d, θ_d)` by reading motor encoder values `(θ_L, θ_R)` and applying the required voltage commands to the right and left motors so that the robot moves towards the desired pose. Additionally, the robot needs to avoid obstacles while proceeding towards the target pose. In summary, there are two primary challenges:
1. Moving from the current pose to the desired pose.
2. Avoiding obstacles during navigation.

---

## **Objectives**  
1. Enable the robot to dynamically navigate to a desired pose (position and orientation) via encoder feedback.  
2. Implement obstacle detection and avoidance capabilities using ultrasonic sensors.  
3. Simulate the system’s behavior in MATLAB/Simulink.  
4. Develop the navigation system for real-world validation using Arduino.  

---

## **Technical Description**  

### **System Architecture**  
The project comprises three main components:

#### **1. Hardware System**  
- **Arduino Uno**: Main controller.  
- **2 DC motors**: For drive propulsion.  
- **H-Bridge L298N**: For motor control.  
- **LiPo Battery**: Provides power to the robot.  
- **1 Ultrasonic Sensor**: For obstacle detection.  
- **1 Servo Motor**: For sensor angular movement.  
- **2 Angular Speed Encoders**: For feedback on wheel rotation.  
- **Chassis**: The robot’s physical structure.  

**Robot Specifications**:
- **Battery Type**: Lithium Iron Phosphate (LiFePO₄), known for its long cycle life, safety, and thermal stability.  
- **Dimensions**: Robot height: 50 cm, diameter: 55 cm.  
- **Wheel Radius**: 12.7 cm, facilitating smooth movement.  
- **Axle Distance (L)**: 500 mm, influencing the turning radius and maneuverability.

#### **2. Software System**  
- **MATLAB/Simulink**: Used for simulating the pose-reaching controller model and system response.  
- **Arduino IDE**: For microcontroller programming and real-time implementation.  
- **Servo Motor Library**: For controlling the servo motor in Arduino.

#### **3. Dynamic Navigation Algorithms**  
- **Simulink Differential Kinematics**: For pose calculation.  
- **Simulink Motors Model**: Simulating motor dynamics.  
- **PID Trajectory Correction**: Based on encoder inputs and desired pose.  
- **Nicolas-Ziegler Algorithm**: For tuning PID parameters.  
- **Arduino Functions**: For regulating voltage commands to left and right motors and implementing obstacle avoidance.

---

### **Workflow**  
1. **Finding Physical Robot Variables**:
   - Torque constant `kt (Nm/A)`
   - Back EMF constant `kb (V/rad/s)`
   - Geared motor inertia `Jm (Kg/m²)`
   - Viscous damping `bm (Nm/rad/s)`
   - Armature resistance `Ra (Ohms)`
   - Armature inductance `La (mH)`

2. **Choosing Hardware**:
   - Select hardware based on robot specifications and power requirements.

3. **Modeling Phase**:
   - Model system dynamics and DC motors.
   - Implement the Nicolas-Ziegler algorithm for tuning PID parameters.

4. **Simulation Phase**:
   - Define target pose and simulate robot motion in MATLAB/Simulink.
   - Test response to various pose configurations.

5. **Implementation Phase**:
   - Develop Arduino algorithm for physical variable input and desired pose.
   - Interface with sensors and motors.

6. **Testing Phase**:
   - Validate the system’s functionality through hardware testing.

---

### **Performance Metrics**  
- **Pose Accuracy**: ±2 cm in position and ±5° in orientation.  
- **Obstacle Clearance**: Maintain a safety distance of at least 10 cm.

---

### **Arduino Workflow Description**  
The Arduino implementation manages sensor readings, control commands, and motor actuation in real-time.

#### **Workflow Logic**:

1. **Encoder Data Processing**:
   - Reads pulse counts from encoders to compute wheel angular velocities (`ω_L, ω_R`).
   - Uses interrupts for precise real-time updates to encoder counts.

2. **Pose Calculation**:
   - Computes the robot’s current pose (`x, y, θ`) using odometry equations.
   - Updates pose periodically based on encoder inputs.

3. **PID Control**:
   - Implements PID controllers for `x`, `y`, and `θ` to minimize errors in position and orientation.
   - Proportional Term: Corrects the error instantly.
   - Integral Term: Accounts for accumulated past errors.
   - Derivative Term: Predicts future errors.
   - Outputs corrected voltage commands for left and right motors based on the PID output.

4. **Obstacle Detection and Avoidance**:
   - Uses an ultrasonic sensor mounted on a servo motor to scan the environment.
   - Measures the distance to obstacles and determines the safest trajectory to avoid collisions.
   - If an obstacle is detected within the threshold distance, the robot halts and adjusts its trajectory dynamically.

5. **Motor Control**:
   - Maps voltage commands to PWM signals for the motor drivers (H-bridge).
   - Adjusts motor direction using IN1, IN2, IN3, IN4 pins and speed via ENA, ENB PWM signals.

6. **Servo Control**:
   - Scans surroundings at predefined angles using the servo motor for ultrasonic sensing.

---

### **Mathematical Modeling for Pose Estimation**  

#### **1. Linear Velocity Calculation**  
To calculate the robot's linear velocity `v`, we take the average of the left (`ω_l`) and right (`ω_r`) wheel velocities:

$v = \frac{R}{2} (\omega_r + \omega_l)$


Where:
- `R` is the radius of the wheels.
- `ω_r` and `ω_l` are the angular velocities of the right and left wheels, respectively.

#### **2. Angular Velocity Calculation**  
To calculate the robot's angular velocity `θ̇`:

\[ \dot{\theta} = \frac{R}{L} (\omega_r - \omega_l) \]

Where:
- `L` is the distance between the left and right wheels (the wheelbase).
- `ω_r` and `ω_l` are the wheel angular velocities.

#### **3. Pose Update Equations**  
To update the robot’s position and orientation, we use the following equations:

- **Position Update**:  
  - In the X-direction:  
    \[ \Delta x = v \cdot \cos(\theta) \cdot \Delta t \]
  - In the Y-direction:  
    \[ \Delta y = v \cdot \sin(\theta) \cdot \Delta t \]

- **Orientation Update**:  
  - In the θ-direction:  
    \[ \Delta \theta = \dot{\theta} \cdot \Delta t \]

#### **4. Final Pose Update Equation**  
The final updated pose after integrating the above components:

- **Position**:  
  \[ x_{new} = x_{old} + v \cdot \cos(\theta) \cdot \Delta t \]  
  \[ y_{new} = y_{old} + v \cdot \sin(\theta) \cdot \Delta t \]  

This model helps track the robot's movements and ensures accurate pose updates for navigation.
