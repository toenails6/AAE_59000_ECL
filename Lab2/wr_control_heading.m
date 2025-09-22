function [wr] = wr_control_heading(wr, time)
    %% Settings. 
    % Default values for zero-radius heading control. 
    wr.DIRL = 0;
    wr.DIRR = 1;
    % PWML = 0;
    % PWMR = 0;

    % Rover heading angle calibration offset. 
    % This is in case that the QTM given current heading is not fully
    % aligned with the rover's physical heading. 
    % Use units in degrees here. 
    theta_offset = 0; 

    % Create PID controller data buffers. 
    if ~isfield(wr, "PID_prev_err")
        wr.PID_prev_err = 0;
    end
    if ~isfield(wr, "PID_integral")
        wr.PID_integral = 0;
    end

    %% PID controller implementation. 
    % Heading vectors and rotation direction processing. 
    % Counter-clockwise as positive heading rotation direction. 
    % Heading rotation direction, heading_sign, is used to control the
    % direction of the rover's rotation, through the wr.DIR* registers.
    heading_unitVec = wr.heading_vec.'/norm(wr.heading_vec); 
    heading_unitDir = wr.heading_dir.'/norm(wr.heading_dir); 
    heading_sign = sign(...
        heading_unitVec(1)*heading_unitDir(2) - ...
        heading_unitVec(2)*heading_unitDir(1)); 
    wr.DIRL = heading_sign < 0;
    wr.DIRR = heading_sign > 0;

    % Proportion error with tolerance margin and calibration offset. 
    % There will be no negative error values here. 
    delta_theta = acos(dot(heading_unitVec, heading_unitDir)) + ...
        deg2rad(theta_offset); 
    error = (abs(delta_theta) > deg2rad(15)) * delta_theta;

    % Integral and integral clamping. 
    wr.PID_integral = wr.PID_integral + error*time.dt;  
    wr.PID_integral = min(max(wr.PID_integral, -90), 90); 

    % Derivative. 
    derivative = (error - wr.PID_prev_err) / time.dt;

    % Assemble PID controller output. 
    u = K_P * error + K_I * wr.PID_integral + K_D * derivative; 

    % Update error history. 
    wr.PID_prev_err = error;

    % Update position history. 
    wr.pos_old = wr.pos; 

    % It seems that the main script file updates the heading vector in the
    % loop. There needs to be no manual heading updates here. 
    % But position history does not seem to be updated in the loop though. 
    % Current position, like current heading, seems to be updated in the
    % loop. 

    % Assign output values. 
    % Equal control efforts on both sides for zero radius heading control. 
    % Opposite directions implemented in the rotation direction processing
    % section. 
    PWML = u; 
    PWMR = u; 

    %% Enforce limits and push control outputs to data struct. 
    PWML = min(90, PWML);
    wr.PWML = uint8(max(0, PWML));
    PWMR = min(90, PWMR);
    wr.PWMR = uint8(max(0, PWMR));
end
