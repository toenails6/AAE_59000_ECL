function [wr] = wr_control_spd(wr, time)
   % defalut values
    wr.DIRL = 1;
    wr.DIRR = 1;
    %PWML =0;
    %PWMR =0;

    % setting the PWM limits, keep the codes here
    P_gain = 0.4;  
    I_gain = 0.0;
    D_gain = 2;

    delta_pos = wr.pos - wr.pos_old; 
    meas_spd = norm(delta_pos)/time.dt;
    disp("Measured Speed: "); 
    disp(meas_spd); 
%     disp("Displacement: "); 
%     disp(disp1); 
%     meas_spd = disp1 / time.dt; 
    speed_error = wr.forward_spd - meas_spd;

    % Update position history. 
    wr.pos_old = wr.pos; 

    if ~isfield(wr, "prev_err")
        wr.prev_err = 0;
    end
    if ~isfield(wr, "int_err")
        wr.int_err = 0;
    end

    wr.int_err = wr.int_err + speed_error * time.dt;
    d_err = (speed_error - wr.prev_err) / time.dt;
    pwm_cmd = P_gain * speed_error + I_gain * wr.int_err + D_gain * d_err;
    wr.prev_err = speed_error;

    PWML = pwm_cmd;
    PWMR = pwm_cmd;

    PWML = min(150, PWML);
    wr.PWML = uint8(max(0, PWML));
    PWMR = min(150, PWMR);
    wr.PWMR = uint8(max(0, PWMR));
end
