function [wr] = wr_control_spd(wr, time)
    %% Settings. 
    % Defalut values for straight velocity control. 
    wr.DIRL = 1;
    wr.DIRR = 1;
    % PWML = 0;
    % PWMR = 0;

    % PID gains. 
    K_P = 0.4;  
    K_I = 0.0;
    K_D = 2;

    % Create PID controller data buffers. 
    if ~isfield(wr, "PID_prev_err")
        wr.PID_prev_err = 0;
    end
    if ~isfield(wr, "PID_integral")
        wr.PID_integral = 0;
    end

    %% PID controller implementation. 
    % Proportion error. 
    meas_spd = norm(wr.pos - wr.pos_old)/time.dt; 
    error = wr.forward_spd - meas_spd;

    % Integral and integral clamping. 
    wr.PID_integral = wr.PID_integral + error * time.dt;  
    wr.PID_integral = min(max(wr.PID_integral, -150), 150); 

    % Derivative. 
    derivative = (error - wr.PID_prev_err) / time.dt;

    % Update error history. 
    wr.PID_prev_err = error;

    % Update position history. 
    wr.pos_old = wr.pos; 

    % Assemble PID controller output. 
    u = K_P * error + K_I * wr.PID_integral + K_D * derivative; 

    % Assign output values. 
    % Equal control efforts on both sides for straight velocity control. 
    PWML = u; 
    PWMR = u; 

    %% Enforce limits and push control outputs to data struct. 
    PWML = min(150, PWML);
    wr.PWML = uint8(max(0, PWML));
    PWMR = min(150, PWMR);
    wr.PWMR = uint8(max(0, PWMR));
end
