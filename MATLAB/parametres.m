% Motor Parameters Definition for DC Motor with Geared Mechanism

% Torque constant (kt) [Nm/A] 
% This is the constant that relates the current to the output torque.
% The higher the kt, the greater the torque produced per unit of current.
kt = 0.062;  % Torque constant for both motors (in Nm/A)

% Back EMF constant (kb) [V/(rad/s)] 
% This is the constant that relates the angular velocity to the back electromotive force (EMF).
% It is usually numerically equal to the torque constant (kt) for many motors.
kb = 0.062;  % Back EMF constant for both motors (in V/(rad/s))

% Geared Motor Inertia (Jm) [Kg/m^2]
% This is the inertia of the motor, which represents the resistance to changes in rotational speed.
% A higher inertia value means it takes more effort to accelerate or decelerate the motor.
Jm = 0.0551;  % Inertia of the motor (in Kg*m^2)

% Viscous Damping (bm) [Nm/(rad/s)]
% This represents the resistance to rotational motion due to friction within the motor.
% Higher values result in slower responses to changes in speed.
bm = 0.188;  % Viscous damping coefficient (in Nm/(rad/s))

% Armature Resistance (Ra) [Ohms]
% The resistance in the motor windings that opposes the flow of current.
% This value contributes to power loss and influences the current needed for a given torque.
Ra = 0.56;  % Armature resistance (in Ohms)

% Armature Inductance (La) [Henrys]
% This is the inductance of the motor's armature windings, which determines the rate of current change.
% It affects the motor's response to voltage changes, especially at high speeds.
La = 0.00097;  % Armature inductance (in Henries)

% Radius of the Wheel (R_wheel) [cm]
% This is the radius of the wheel connected to the motor, which affects the linear velocity.
R_wheel = 12.7;  % Radius of the wheel (in cm)

% Axle Distance (L) [mm]
% This is the distance between the two wheels in a differential drive robot.
% The axle distance impacts the turning radius and maneuverability of the robot.
L = 500;  % Axle distance (in mm)

% Displaying the Parameters
disp('Motor Parameters:')
disp(['Torque constant (kt): ', num2str(kt), ' Nm/A'])
disp(['Back EMF constant (kb): ', num2str(kb), ' V/(rad/s)'])
disp(['Geared Motor Inertia (Jm): ', num2str(Jm), ' Kg*m^2'])
disp(['Viscous Damping (bm): ', num2str(bm), ' Nm/(rad/s)'])
disp(['Armature Resistance (Ra): ', num2str(Ra), ' Ohms'])
disp(['Armature Inductance (La): ', num2str(La), ' Henries'])
disp(['Radius of the Wheel (R_wheel): ', num2str(R_wheel), ' cm'])
disp(['Axle Distance (L): ', num2str(L), ' mm'])
