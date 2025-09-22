function [wr] = wr_control_heading(wr, time)
    %% Settings. 
    % Defalut values for zero-radius heading control. 
    wr.DIRL = 0;
    wr.DIRR = 1;
    % PWML = 0;
    % PWMR = 0;

    % Rover heading angle calibration offset. 
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
    heading_unitVec = wr.heading_vec.'/norm(wr.heading_vec); 
    heading_unitDir = wr.heading_dir.'/norm(wr.heading_dir); 
    heading_sign = sign(...
        heading_unitVec(1)*heading_unitDir(2) - ...
        heading_unitVec(2)*heading_unitDir(1)); 
    wr.DIRL = heading_sign < 0;
    wr.DIRR = heading_sign > 0;

    % Proportion error with tolerance margin and calibration offset. 
    delta_theta = acos(dot(heading_unitVec, heading_unitDir)) + ...
        theta_offset; 
    error = (abs(delta_theta) > deg2rad(1.5)) * delta_theta;

    integral = integral + error * Ts;  % Simple integral (add clamping for anti-windup if needed)
    derivative = (error - prev_error) / Ts;
    u = Kp * error + Ki * integral + Kd * derivative;
    prev_error = error;

    %% Enforce limits and push control outputs to data struct. 
    PWML = min(90, PWML);
    wr.PWML = uint8(max(0, PWML));
    PWMR = min(90, PWMR);
    wr.PWMR = uint8(max(0, PWMR));
end
