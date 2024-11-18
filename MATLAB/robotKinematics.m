function [v_x_global, v_y_global, angular_velocity] = robotKinematics(R, L, theta, omega_r, omega_l)
    % Inputs:
    % R: Radius of the wheels (meters)
    % L: Distance between the wheels (meters)
    % theta: Current orientation of the robot (in radians)
    % omega_r: Angular velocity of the right wheel (rad/s)
    % omega_l: Angular velocity of the left wheel (rad/s)
    
    % Transformation matrix
    T = [R/2 * cos(theta), R/2 * cos(theta);
         R/2 * sin(theta), R/2 * sin(theta);
         R/L,            -R/L];
     
    % Wheel angular velocity vector
    omega = [omega_r; omega_l];
    
    % Compute velocity components
    q_i = T * omega;
    
    % Extract components
    v_x_global = q_i(1);          % Global linear velocity in x-direction
    v_y_global = q_i(2);          % Global linear velocity in y-direction
    angular_velocity = q_i(3);  % Angular velocity (theta_dot)
end

