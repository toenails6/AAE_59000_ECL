function wr = wr_control_wp(wr, time)

    % initialize waypoint index
    if ~isfield(wr, 'curWP')
        wr.curWP = 1;
    end

    % check if all waypoints are done
    if wr.curWP > size(wr.WP,1)
        wr.PWML = 0;
        wr.PWMR = 0;
        return
    end

    % current waypoint
    wp = wr.WP(wr.curWP,1:2);

    % distance to waypoint
    dist = norm(wp(:) - wr.pos(:));

    % if close enough, switch to next waypoint
    if dist < wr.dist_mar
        wr.curWP = wr.curWP + 1;
        if wr.curWP > size(wr.WP,1)
            wr.PWML = 0;
            wr.PWMR = 0;
            return
        end
        wp = wr.WP(wr.curWP,1:2);
    end

    % desired heading toward waypoint
    wr.heading_dir = wp(:) - wr.pos(:);

    % heading error
    v = wr.heading_vec.'/norm(wr.heading_vec);   % current heading
    d = wr.heading_dir.'/norm(wr.heading_dir);   % desired heading
    a = acos(max(-1,min(1, dot(v,d))));
    s = v(1)*d(2) - v(2)*d(1);
    ang_err = sign(s)*a;

    % decide control
    if abs(ang_err) < deg2rad(wr.fwd_deg)
        wr = wr_control_spd(wr, time);
    else
        wr = wr_control_heading(wr, time);
    end
end
