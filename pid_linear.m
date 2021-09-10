function [v, e] = pid_linear(stamped_waypoints, t_ref)
%   Linear PID: Calculates the linear control
%   Using the difference between the time of the latest 
%   timestamp and the newest, we get an error signal. This
%   signal is compared against a time buffer reference,
%   and a control signal is generated.

% Specify proportional constant
Kp = 0.2;

% Calculate time difference
t1 = stamped_waypoints(1,3);
t2 = stamped_waypoints(end, 3);
dt = t2 - t1;

% Convert into error, by comparing with reference
e = dt - t_ref;

% Calculate control signal
if(e > 0)
    v = Kp * e;
    if(v > 0.35)
        v = 0.35;
    end
else
    v = 0;
end

