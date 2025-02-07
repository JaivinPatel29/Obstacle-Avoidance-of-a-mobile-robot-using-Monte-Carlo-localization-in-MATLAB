clc; 
clear;
openExample('robotics/PathPlanningExample');
load exampleMaps.mat;
% Define robot parameters
robot = differentialDriveKinematics('WheelRadius', 0.05, 'WheelSpeedRange', [-inf, inf], 'TrackWidth', 0.5, 'VehicleInputs', 'VehicleSpeedHeadingRate');
initialPose = [3; 0; pi/2];
tend = 30; % End time
dt = 0.1; % Time step
t = 0:dt:tend;
pose(:, 1) = initialPose;%Define map and obstacles
map = binaryOccupancyMap(simpleMap, 2); % 10x10 map with 2 increments
setOccupancy(map, [2 2; 3 4; 5 2; 8 7; 2 9], 1); % Set obstacle coordinates
% Define sensor parameters
sensor = rangeSensor('Range', [0.1 10], 'HorizontalAngle', [-pi/3 pi/3]);
safeDistance = 0.5;
v = 0.5; 
omega = 0;
% Visualization setup
figure;
TrackWidth = 0.5;
framesize = 0.8 * TrackWidth;
vizRate = rateControl(1/dt);
%Defining Parameters for Monte Carlo Algorithm
odometryModel=odometryMotionModel
odometryModel.Noise=[0.2 0.2 0.2 0.2];
rangeFinderModel=likelihoodFieldSensorModel
rangeFinderModel.SensorLimits=[0.45 8];
rangeFinderModel.Map=map;
numParticles=1000;
initialCovariance=eye(3);
mcl=monteCarloLocalization('InitialPose',initialPose,'InitialCovariance',initialCovariance,'ParticleLimits',[500 numParticles]);
mcl.MotionModel=odometryModel;
mcl.SensorModel=rangeFinderModel;
mcl.GlobalLocalization=false;
mcl.UseLidarScan=true;
for i = 2:length(t) % Starting from 2 because we want to use initial pose
    % Get sensor readings
    [ranges, angles] = sensor(pose(:, i-1)' + [0.4 0.4 pi/2], map);
    
    % Obstacle avoidance
    if min(ranges) < safeDistance
        omega = -0.5; % If obstacle is within safe distance, turn the robot
    else
        omega = 0; % Reset omega if no obstacle is near
    end
      % Update robot pose
    vel = derivative(robot, pose(:, i-1)', [v omega]); % vel is derivative of xdot, ydot, thetadot
    pose(:, i) = pose(:, i-1) + vel * dt;
    
   % Plot robot position and map
    show(map);
    hold off;
    T1 = [pose(1, i); pose(2, i); 0];
    plotRot = axang2quat([0 0 1 pose(3, i)]);
    plotTransforms(T1', plotRot, 'MeshFilePath', 'groundvehicle.stl', 'Parent', gca, 'View', '2D', 'FrameSize', framesize);
    light;
    xlim([0 13]);
    ylim([0 13]);
    waitfor(vizRate);
end
% Plot the robot's path
figure;
show(map);
hold on;
plot(pose(1, :), pose(2, :), 'r');
hold off;
xlabel('X [m]');
ylabel('Y [m]');
title('Robot Path with Obstacle Avoidance');
grid on;
