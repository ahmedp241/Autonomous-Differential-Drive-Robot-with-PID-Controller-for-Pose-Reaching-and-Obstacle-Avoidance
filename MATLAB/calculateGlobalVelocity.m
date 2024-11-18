function global_velocity  = calculateGlobalVelocity(v_x_global, v_y_global)
    % Inputs:
    % x_global: Global linear velocity in x-direction (m/s)
    % y_global: Global linear velocity in y-direction (m/s)
    
    % Compute the magnitude of the global velocity
    global_velocity = sqrt(v_x_global^2 + v_y_global^2);
    
   
end

