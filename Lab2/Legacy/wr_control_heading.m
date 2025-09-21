function [wr] = wr_control_heading(wr, time)

   % % defalut values
   %  wr.DIRL = 1;
   %  wr.DIRR = 1;
   %  PWML =0;
   %  PWMR =0;
   % 
   %  % setting the PWM limits, keep the codes here
   %  PWML = min(150, PWML);
   %  wr.PWML = uint8(max(0, PWML));
   %  PWMR = min(150, PWMR);
   %  wr.PWMR = uint8(max(0, PWMR));
    P_gain = 1;  
    I_gain = 1;
    D_gain = 1;

    % --- Compute heading error in degrees ---
    % Normalize vectors just in case
    hv = wr.heading_vec / norm(wr.heading_vec);
    dv = wr.heading_dir / norm(wr.heading_dir);

    % Angle between heading and desired vector
    angle_err = acos(dot(hv,dv));      % radians
    angle_err = rad2deg(angle_err);     % degrees

    % Determine sign of error using 2D cross product (z-component)
    cross_val = hv(1)*dv(2) - hv(2)*dv(1);
    if cross_val > 0
        % heading vector is rotated CCW from desired → deviation to left
        sign_err = +1;  % positive error
    else
        % deviation to right
        sign_err = -1;
    end
    heading_error = sign_err * angle_err;  % signed heading error in degrees

    persistent int_err prev_err
    if isempty(int_err)
        int_err = 0;
    end
    if isempty(prev_err)
        prev_err = 0;
    end

    int_err = int_err + heading_error * time.dt;
    d_err = (heading_error - prev_err) / time.dt;

    pwm_cmd = abs(P_gain * heading_error + I_gain * int_err + D_gain * d_err);
    prev_err = heading_error;

    %Check heading margin
    if abs(heading_error) < wr.fwd_deg
        %heading achieved
        pwm_cmd = 0;
    end

    %Decide wheel directions based on sign of error
    if heading_error > 0
        % deviated left → turn right: left forward, right backward
        wr.DIRL = 1;
        wr.DIRR = 0;
    elseif heading_error < 0
        % deviated right → turn left: left backward, right forward
        wr.DIRL = 0;
        wr.DIRR = 1;
    else
        % perfect heading
        wr.DIRL = 1;
        wr.DIRR = 1;
    end

    wr.PWML = uint8(pwm_cmd);
    wr.PWMR = uint8(pwm_cmd);
    PWML = min(150, PWML);
    wr.PWML = uint8(max(0, PWML));
    PWMR = min(150, PWMR);
    wr.PWMR = uint8(max(0, PWMR));
end