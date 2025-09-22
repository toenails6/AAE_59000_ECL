function wr = wr_control_wp(wr, time)
    persistent WP numWP
    if isempty(WP)
        file  = 'Rover_wp_lab2.xlsx';
        sheet = 'white_rover';
        WP = readmatrix(file,'Sheet',sheet);
        WP = WP(:,1:2);
        numWP = size(WP,1);
    end

    if wr.curWP > numWP
        wr.DIRL=1;
        wr.DIRR=1;
        wr.PWML=uint8(0);
        wr.PWMR=uint8(0);
        return
    end

    wp   = WP(wr.curWP,1:2);
    dist = norm(wp(:) - wr.pos(:));

    if dist < wr.dist_mar
        wr.curWP = wr.curWP + 1;
    end

    % heading error
    v = wr.heading_vec.'/norm(wr.heading_vec);
    d = wr.heading_dir.'/norm(wr.heading_dir);
    a = acos(max(-1,min(1, dot(v,d))));
    s = v(1)*d(2) - v(2)*d(1);
    ang_err = sign(s)*a;
    if abs(ang_err) < deg2rad(wr.fwd_deg)
        ang_err = 0;
    end

    if ~isfield(wr,'PIDv_prev_err')
        wr.PIDv_prev_err = 0;
    end
    if ~isfield(wr,'PIDv_integral')
        wr.PIDv_integral = 0;
    end
    if ~isfield(wr,'PIDh_integral')
        wr.PIDh_integral = 0;
    end
    if ~isfield(wr,'PIDh_prev_err')
        wr.PIDh_prev_err = 0;
    end

    % forward speed
    Kp_v=0.40; Ki_v=0.00; Kd_v=2.0;

    meas_spd = norm(wr.pos - wr.pos_old)/time.dt;
    error = wr.forward_spd - meas_spd;
    wr.PIDv_integral = wr.PIDv_integral + error*time.dt;
    wr.PIDv_integral = min(max(wr.PIDv_integral, -150), 150);
    derivative = (error - wr.PIDv_prev_err) / time.dt;
    u_f = Kp_v*error + Ki_v*wr.PIDv_integral + Kd_v*derivative;
    wr.PIDv_prev_err = error;

    % heading
    Kp_h=90; Ki_h=0.00; Kd_h=0.0;

    wr.PIDh_integral = wr.PIDh_integral + ang_err*time.dt;
    wr.PIDh_integral = min(max(wr.PIDh_integral, -150), 150);
    d_ang = (ang_err - wr.PIDh_prev_err) / time.dt;
    u_turn = Kp_h*ang_err + Ki_h*wr.PIDh_integral + Kd_h*d_ang;
    wr.PIDh_prev_err = ang_err;
    u_turn = max(-steer_max, min(steer_max, u_turn));

    L = u_f - u_turn;
    R = u_f + u_turn;

    if L >= 0
        wr.DIRL = 1;
        Lp = min(150,round(L));
    else
        wr.DIRL = 0;
        Lp = min(150,round(-L));
    end
    if R >= 0
        wr.DIRR = 1;
        Rp = min(150,round(R));
    else
        wr.DIRR = 0;
        Rp = min(150,round(-R));
    end

    wr.PWML = uint8(max(0,Lp));
    wr.PWMR = uint8(max(0,Rp));

    wr.pos_old = wr.pos;
end
