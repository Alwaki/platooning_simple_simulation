%% FORMALIA
%
% Brief Description:        Code for testing a PID controller for
%                           platooning.
%
% Author:                   Alexander Wallen Kiessling
%
% Extended Description:     This algorithm uses a lead vehicle
%                           that follows specified goal points along
%                           a trajectory, using the pure pursuit algorithm.
%                           The poses of this vehicle are then periodically
%                           saved in an array along with a correspondiong 
%                           timestamp. A following vehicle attempts to
%                           follow these saved poses, in the order they
%                           are saved. To regulate distance between the
%                           lead and following vehicle, the difference
%                           in time between the first and last pose is
%                           used as an error. Furthermore, when the
%                           following vehicle gets within a certain
%                           tolerance distance from these waypoints, it is
%                           removed from the stored list. 
%
% Todo:                     Increase complexity of controller.

%% SETUP

% Clean environment
clc;
clear all;
close all;

% Parameters
goalRadius         = 0.1;
sampleTime         = 0.1;
t_ref              = 5;
time               = 0;
waypoint_tolerance = 0.8;
path               = [2 2; 2.5 4; 4 6; 5 8; 7 9; 
                      9 11; 12 10; 12 9; 11.5 8;
                      7 6];

%% CODE
% Create vehicle model objects
leader = differentialDriveKinematics("TrackWidth", 1, "VehicleInputs", "VehicleSpeedHeadingRate", ...
                                    "WheelRadius", 0.05);
                                
robot = differentialDriveKinematics("TrackWidth", 1, "VehicleInputs", "VehicleSpeedHeadingRate", ...
                                    "WheelRadius", 0.05);

robot2 = differentialDriveKinematics("TrackWidth", 1, "VehicleInputs", "VehicleSpeedHeadingRate", ...
                                    "WheelRadius", 0.05);
% Initialize leader object
leaderInitialLocation = path(1,:);
leaderGoal = path(end,:);
initialOrientation = 0;
leaderCurrentPose = [leaderInitialLocation initialOrientation]';

% Intialize follower object
robotInitialLocation = [2 1];
robotGoal = path(end,:);
initialOrientation = 0;
robotCurrentPose = [robotInitialLocation initialOrientation]';

% Intialize follower object
robot2InitialLocation = [2 0];
robot2Goal = path(end,:);
initialOrientation = 0;
robot2CurrentPose = [robotInitialLocation initialOrientation]';

% Pure pursuit controller
    controller1 = controllerPurePursuit;
    controller1.Waypoints = path;
    controller1.DesiredLinearVelocity = 0.3;
    controller1.MaxAngularVelocity = 2;
    controller1.LookaheadDistance = 0.3;

%% PLOTTING
distanceToGoal = norm(robotInitialLocation - robotGoal);

% Initialize the simulation loop
vizRate = rateControl(1/sampleTime);

% Initialize the figure
figure

% Determine vehicle frame size to most closely represent vehicle with plotTransforms
frameSize = robot.TrackWidth/0.8;

stamped_waypoints1 = [];
stamped_waypoints2 = [];

while(distanceToGoal > goalRadius )

    % Append waypoint array with leader pose and timestamp
    stamped_waypoints1 = [stamped_waypoints1; leaderCurrentPose(1:2).' time];
    stamped_waypoints2 = [stamped_waypoints2; robotCurrentPose(1:2).' time];
    
    % Compute the leader controller outputs
    [v1, omega1] = controller1(leaderCurrentPose);
    
    % Compute follower 1 controller outputs
    [v2, ~] = pid_linear(stamped_waypoints1, t_ref);
    y = stamped_waypoints1(1,2)- robotCurrentPose(2);
    x = stamped_waypoints1(1,1) - robotCurrentPose(1);
    theta = atan2(y,x);
    dtheta = theta - robotCurrentPose(3);
    dtheta = mod(dtheta + pi, 2 * pi) - pi;
    omega2 = pid_angular(dtheta);
    
    % Compute follower 2 controller outputs
    [v3, e] = pid_linear(stamped_waypoints2, t_ref);
    disp(v3)
    y2 = stamped_waypoints2(1,2)- robot2CurrentPose(2);
    x2 = stamped_waypoints2(1,1) - robot2CurrentPose(1);
    theta2 = atan2(y2,x2);
    dtheta2 = theta2 - robot2CurrentPose(3);
    dtheta2 = mod(dtheta2 + pi, 2 * pi) - pi;
    omega3 = pid_angular(dtheta2);
    
    % Get the robot's velocity using controller inputs
    vel1 = derivative(leader, leaderCurrentPose, [v1 omega1]);
    vel2 = derivative(robot, robotCurrentPose, [v2 omega2]);
    vel3 = derivative(robot2, robot2CurrentPose, [v3 omega3]);
    
    % Update the current poses
    leaderCurrentPose = leaderCurrentPose + vel1*sampleTime; 
    robotCurrentPose = robotCurrentPose + vel2*sampleTime; 
    robot2CurrentPose = robot2CurrentPose + vel3*sampleTime; 
    
    
    % Re-compute the distance to the goal
    distanceToGoal = norm(leaderCurrentPose(1:2) - leaderGoal(:));
    
    % Remove passed waypoints for follower 1
    for i = 1:length(stamped_waypoints1(1))
        distanceToWaypoint = norm(robotCurrentPose(1:2) - stamped_waypoints1(i,1:2).');
        if(distanceToWaypoint < waypoint_tolerance)
            stamped_waypoints1(1,:) = [];
        end
    end
    
    % Remove passed waypoints for follower 2
    for i = 1:length(stamped_waypoints2(1))
        distanceToWaypoint2 = norm(robot2CurrentPose(1:2) - stamped_waypoints2(i,1:2).');
        if(distanceToWaypoint2 < waypoint_tolerance)
            stamped_waypoints2(1,:) = [];
        end
    end
    
    % Update the plot
    hold off
    
    % Plot path each instance so that it stays persistent while robot mesh
    % moves
    plot(path(:,1), path(:,2),"k--d")
    hold all
    
    % Plot the path of the robot as a set of transforms
    plotTrVec = [leaderCurrentPose(1:2); 0];
    plotRot = axang2quat([0 0 1 leaderCurrentPose(3)]);
    plotTransforms(plotTrVec', plotRot, "MeshFilePath", "groundvehicle.stl", "Parent", gca, "View","2D", "FrameSize", frameSize);
    plotTrVec = [robotCurrentPose(1:2); 0];
    plotRot = axang2quat([0 0 1 robotCurrentPose(3)]);
    plotTransforms(plotTrVec', plotRot, "MeshFilePath", "groundvehicle.stl", "Parent", gca, "View","2D", "FrameSize", frameSize);
    plotTrVec = [robot2CurrentPose(1:2); 0];
    plotRot = axang2quat([0 0 1 robot2CurrentPose(3)]);
    plotTransforms(plotTrVec', plotRot, "MeshFilePath", "groundvehicle.stl", "Parent", gca, "View","2D", "FrameSize", frameSize);
    plot(stamped_waypoints1(:,1), stamped_waypoints1(:,2), 'rx')
    plot(stamped_waypoints2(:,1), stamped_waypoints2(:,2), 'bx')
    light;
    xlim([0 13])
    ylim([0 13])

    waitfor(vizRate);
    
    % Update the time
    time = time + vizRate.LastPeriod;
end