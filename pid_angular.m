function [output_control, error] = pid_angular(dtheta, prev_error, prev_control)
%   Angular PID: Calculates the angular control
%   This function will calculate the angular velocity control for 
%   a given difference in angles.

% Saturation limits (model-specific)
limit = 1.82;

% Specify constants
Kp = 2;
Ki = 0.2;
T = 0.1;

% Rename input parameter
error = dtheta;

% Calculate control output
output_control = Kp * error; %+ prev_control + Ki * (error + prev_error)* T / 2;

% Limit output
if(output_control > limit)
    output_control = limit;
elseif(output_control < -limit)
    output_control = -limit;
end

end



