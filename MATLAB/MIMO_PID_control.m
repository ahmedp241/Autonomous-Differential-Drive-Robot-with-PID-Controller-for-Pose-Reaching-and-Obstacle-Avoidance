function [uLEFT, uRIGHT] = MIMO_PID_control(u_x, u_y, u_theta, theta)
    % MIMO PID Control for a differential drive robot
    % Inputs:
    % u_x      - Desired linear velocity in the x direction
    % u_y      - Desired linear velocity in the y direction
    % u_theta  - Desired angular velocity (theta)
    % r        - Wheel radius
    % L        - Distance between wheels (wheelbase)
    % theta    - Current robot orientation (theta)
    R=0.5;
    L=0.127;
    % Compute the motor velocities based on kinematics
    % Using the derived equations:
    % u1 = (1/2) * ( (2/r) * (u_x * cos(theta) + u_y * sin(theta)) + (L/r) * u_theta )
    % u2 = (1/2) * ( (2/r) * (u_x * cos(theta) + u_y * sin(theta)) - (L/r) * u_theta )
    
    % Term for the linear velocities in the x and y directions
    linear_term = (2 / R ) * (u_x * cos(theta) + u_y * sin(theta));
    
    % Compute the left and right motor velocities
    uLEFT = 0.5 * (linear_term + (L / R ) * u_theta);
    uRIGHT = 0.5 * (linear_term - (L / R ) * u_theta);
    
    % Return the calculated motor inputs
end

