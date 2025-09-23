function [wr] = wr_control_spd(wr, time)
% Defalut values for straight velocity control. 
wr.DIRL = 1;
wr.DIRR = 1;

% Disregard missing measurements from MOCAP. 
if ~isnan(wr.pos(1)) 
    %% Settings. 
    % PID gains. 
    % Note here that I suggest manually tuning K_P, simply feel it, I might
    % very likely tune this myself. 
    % I suggest to begin with a small integral gain K_I. 
    % I suggest likely relatively moderate (higher than K_I) K_D. 
    % Error in this controller can be negative, which might create
    % oscillations, but higher K_D can help alleviate that. 
    % Error has units of mm/s. 
    K_P = 0.08;  
    K_I = 0.3;
    K_D =0.02;

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
    wr.PID_integral = wr.PID_integral + error*time.dt;  
    % wr.PID_integral = min(max(wr.PID_integral, -150), 150); 

    % Derivative. 
    derivative = (error - wr.PID_prev_err) / time.dt;
    % derivative = min(max(derivative, -150), 150); 

    % Assemble PID controller output. 
    u = K_P * error + K_I * wr.PID_integral + K_D * derivative; 

    % Update error history. 
    wr.PID_prev_err = error;

    % Update position history. 
    wr.pos_old = wr.pos; 

    % Assign output values. 
    % Equal control efforts on both sides for straight velocity control. 
    PWML = u; 
    PWMR = u; 

    % Tuning display. 
    disp("--------------------------------"); 
    disp("Output: "); 
    disp(u); 

    disp("Error: "); 
    disp(error);

    disp("Current position: "); 
    disp(norm(wr.pos));

    disp("Measured speed: "); 
    disp(meas_spd);

    disp("Integral: "); 
    disp(wr.PID_integral);

    disp("Derivative: "); 
    disp(derivative);

    disp("--------------------------------"); 
    
    %% Enforce limits and push control outputs to data struct. 
    PWML = min(150, PWML);
    wr.PWML = uint8(max(0, PWML));
    PWMR = min(150, PWMR);
    wr.PWMR = uint8(max(0, PWMR));
end
end