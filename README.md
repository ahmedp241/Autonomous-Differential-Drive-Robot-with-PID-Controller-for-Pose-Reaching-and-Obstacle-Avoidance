# **Autonomous Differential Drive Robot with PID Controller for Pose Reaching and Obstacle Avoidance**  
**Using Arduino and MATLAB/Simulink Simulation**

---

## **Abstract**  
This project focuses on the design and implementation of a differential-drive autonomous robot. The robot uses a PID controller for pose control, ensuring precise movement and orientation, alongside a distinct obstacle avoidance algorithm to navigate around obstacles. The control systems are integrated on an Arduino platform, with real-time feedback from sensors. Simulink models and MATLAB simulations are used to model and analyze the system, incorporating the Nichols-Ziegler method for tuning the controllers. The repository offers a comprehensive guide to hardware setup, software implementation, control algorithm design, and performance analysis for autonomous robot navigation.

---

## **Problem Statement**  
The goal is to move a robot from its current pose `(x, y, θ)` to a desired pose `(x_d, y_d, θ_d)` by reading motor encoder values `(θ_L, θ_R)` and applying the required voltage commands to the right and left motors so that the robot moves towards the desired pose. Additionally, the robot needs to avoid obstacles while proceeding towards the target pose. In summary, there are two primary challenges:
1. Moving from the current pose to the desired pose.
2. Avoiding obstacles during navigation.

![rrrrr](https://github.com/user-attachments/assets/c52a0cde-7877-43fa-859a-25e6c46e1c61)


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

  ![circuit_image](https://github.com/user-attachments/assets/c65ce24e-d90a-46b6-bbab-fab3e76405d3)


**Robot Specifications**:
- **Battery Type**: Lithium Iron Phosphate (LiFePO₄), known for its long cycle life, safety, and thermal stability.  
- **Dimensions**: Robot height: 50 cm, diameter: 55 cm.  
- **Wheel Radius**: 12.7 cm, facilitating smooth movement.  
- **Axle Distance (L)**: 500 mm, influencing the turning radius and maneuverability.
  ![image](https://github.com/user-attachments/assets/5e4d70cd-4f03-46ce-afd6-6b70cd64701b)


#### **2. Software System**  
- **MATLAB/Simulink**: Used for simulating the pose-reaching controller model and system response.  
- **Arduino IDE**: For microcontroller programming and real-time implementation.  
- **Servo Motor Library**: For controlling the servo motor in Arduino.(Servo.h)

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
  
# Ziegler-Nichols PID Tuning Method

The **Ziegler-Nichols** method is a widely used approach for tuning PID controllers, which calculates the optimal PID gains based on the ultimate gain (**KU**) and the oscillation period (**TU**) observed in the system. The method provides different PID configurations for various types of systems, ensuring effective control performance.

## Controller Types and PID Gains

## Controller Types and PID Gains

The following table summarizes the PID gains for different controller types based on the Ziegler-Nichols method. The formulas for each type of controller are calculated using the **KU** (ultimate gain) and **TU** (oscillation period) values.

| **Controller Type**       | **Description**                                               | **KP**          | **TI**           | **TD**           |
|---------------------------|---------------------------------------------------------------|-----------------|------------------|------------------|
| **ClassicPID**             | Standard PID controller with typical settings for balanced control. | 0.6 * KU        | TU / 2           | TU / 8           |
| **P**                      | Only Proportional control (no integral or derivative action).  | 0.5 * KU        | NaN              | NaN              |
| **PI**                     | Proportional-Integral controller for systems that need integral action. | 0.45 * KU       | TU / 1.2         | NaN              |
| **PD**                     | Proportional-Derivative controller for systems needing derivative action. | 0.8 * KU        | NaN              | TU / 8           |
| **PessenIntegrationRule**  | A variant rule for more aggressive tuning.                    | 0.7 * KU        | 2 * TU / 5       | 3 * TU / 20      |
| **SomeOvershoot**          | PID tuning for systems where some overshoot is acceptable.    | KU / 3          | TU / 2           | TU / 3           |
| **NoOvershoot**            | PID tuning for systems where minimal overshoot is desired.   | 0.2 * KU        | TU / 2           | TU / 3           |



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


![repere](https://github.com/user-attachments/assets/1da55052-ee9a-498a-aa67-a32a082c8279)


#### **1. Linear Velocity Calculation**  
To calculate the robot's linear velocity `v`, we take the average of the left (`ω_l`) and right (`ω_r`) wheel velocities:

$v = \frac{R}{2} (\omega_r + \omega_l)$


Where:
- `R` is the radius of the wheels.
- `ω_r` and `ω_l` are the angular velocities of the right and left wheels, respectively.

#### **2. Angular Velocity Calculation**  
To calculate the robot's angular velocity `θ̇`:

$\[ \dot{\theta} = \frac{R}{L} (\omega_r - \omega_l) \]$

Where:
- `L` is the distance between the left and right wheels (the wheelbase).
- `ω_r` and `ω_l` are the wheel angular velocities.

#### **3. Pose Update Equations**  
To update the robot’s position and orientation, we use the following equations:

- **Position Update**:  
  - In the X-direction:  
    $\[ \Delta x = v \cdot \cos(\theta) \cdot \Delta t \]$
  - In the Y-direction:  
    $\[ \Delta y = v \cdot \sin(\theta) \cdot \Delta t \]$

- **Orientation Update**:  
  - In the θ-direction:  
    $\[ \Delta \theta = \dot{\theta} \cdot \Delta t \]$

#### **4. Final Pose Update Equation**  
The final updated pose after integrating the above components:

- **Position**:  
  $\[ x_{new} = x_{old} + v \cdot \cos(\theta) \cdot \Delta t \] $ 
  $\[ y_{new} = y_{old} + v \cdot \sin(\theta) \cdot \Delta t \] $ 

This model helps track the robot's movements and ensures accurate pose updates for navigation.
