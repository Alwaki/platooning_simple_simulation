function [control_output, error] = pid_linear(stamped_waypoints, t_ref)
%   Linear PID: Calculates the linear control
%   Using the difference between the time of the latest 
%   timestamp and the newest, we get an error signal. This
%   signal is compared against a time buffer reference,
%   and a control signal is generated.

% Saturation limit (model-specific)
limit = 0.26;

% Specify constants
Kp = 0.2;

% Calculate time difference
t1 = stamped_waypoints(1,3);
t2 = stamped_waypoints(end, 3);
dt = t2 - t1;

% Convert into error, by comparing with reference
error = dt - t_ref;

% Calculate control signal
control_output = Kp * error;
if(control_output > limit)
   control_output = limit;
elseif(control_output < 0)
    control_output = 0;
end
end


