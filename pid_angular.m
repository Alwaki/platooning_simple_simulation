function [omega] = pid_angular(dtheta)
%   Angular PID: Calculates the angular control
%   This function will calculate the angular velocity control for 
%   a given difference in angles.

% Specify proportional constant
Kp = 2;

% Calculate control output
omega = Kp * dtheta;

% Limit output
if(omega > 2)
    omega = 2;
elseif(omega < -2)
    omega = -2;
end

end

