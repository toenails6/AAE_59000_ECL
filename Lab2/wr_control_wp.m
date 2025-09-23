function [wr] = wr_control_wp(wr, time)

    if wr.curWP > size(wr.WP,1)
        % no more waypoints: stop
        wr.PWML = uint8(0);
        wr.PWMR = uint8(0);
        return
    end

    if ~isfield(wr, "heading_dir")
        wr.heading_dir = [0,0];
    end

    target = wr.WP(wr.curWP,:).';  % 2x1 target point

    % --- Compute desired heading vector toward target ---
    vec_to_wp = target' - wr.pos;           % 2x1 vector
    dist_to_wp = norm(vec_to_wp);          % distance to waypoint
    desired_heading = vec_to_wp / dist_to_wp; % unit vector toward WP
    disp(target)
    disp(wr.pos)
    % Update desired heading in wr

    wr.heading_dir = desired_heading;

    % --- Compute heading error in degrees ---
    % Heading vectors and rotation direction processing. 
    % Counter-clockwise as positive heading rotation direction. 
    % Heading rotation direction, heading_sign, is used to control the
    % direction of the rover's rotation, through the wr.DIR* registers.
    hv = wr.heading_vec.'/norm(wr.heading_vec); 
    dv = wr.heading_dir.'/norm(wr.heading_dir); 
    angle_err = acos(dot(hv, dv));
    angle_err = rad2deg(angle_err);

    % sign of error (left vs. right)
    cross_val = hv(1)*dv(2) - hv(2)*dv(1);
    if cross_val > 0
        sign_err = +1;
    else
        sign_err = -1;
    end
    heading_error = sign_err * angle_err;

    % --- Waypoint reached? ---
    if dist_to_wp < wr.dist_mar
        wr.curWP = wr.curWP + 1;  % go to next waypoint
        if wr.curWP > size(wr.WP,1)
            % finished all waypoints
            wr.PWML = uint8(0);
            wr.PWMR = uint8(0);
            return
        else
            % new target
            target = wr.WP(wr.curWP,:).';
        end
    end

    % --- Decide mode: heading control or speed control ---
    if abs(heading_error) > wr.fwd_deg
        % rotate first: call heading controller
        wr = wr_control_heading(wr, time); 
    else
        % heading good: move forward with speed control
        wr.forward_spd = wr.forward_spd; % ensure forward speed is set
        wr = wr_control_spd(wr, time);
    end
end