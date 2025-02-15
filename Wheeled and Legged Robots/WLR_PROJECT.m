clc;
clear variables;
load exampleMaps.mat;

robot = differentialDriveKinematics('WheelRadius', 0.05, 'WheelSpeedRange', [-Inf Inf], 'TrackWidth', 0.5, 'VehicleInputs', 'VehicleSpeedHeadingRate');

initialPose = [0; 0; 0];
target = [8; 2];
distToTarget = sqrt((initialPose(1) - target(1))^2 + (initialPose(2) - target(2))^2);
slope = (target(2) - initialPose(2)) / (target(1) - initialPose(1));
initialPose(3) = atan(slope);

% Parameters
T = 25;               
dt = 0.1;             
pose(:, 1) = initialPose;

%binary occupancy map
% map = binaryOccupancyMap(10, 10, 10);
map = binaryOccupancyMap(simpleMap, 2);
setOccupancy(map, [1 3;2 5;4 8;3 1.5;4.5 3;6 8], 1);

%range sensor
sensor = rangeSensor("Range", [0.1 10], "HorizontalAngle", [-pi/3, pi/3]);

% Hybrid A* planner for path optimization
stateValidator = validatorOccupancyMap;
stateValidator.Map = map;
stateValidator.ValidationDistance = 0.01;
planner = plannerHybridAStar(stateValidator, 'MinTurningRadius', 2);
start = [1 1 pi/4];
goal = [8 2 0];
path = plan(planner, start, goal);
inpath = path.States;
% Optimization options
options = optimizePathOptions;
options.MinTurningRadius = 2;
options.MaxPathStates = size(inpath,1) * 3;
options.ObstacleSafetyMargin = 0.75;
optpath = optimizePath(inpath, map, options);

% Parameters for navigation and visualization
safeDistance = 1;
v = 0.5;
omega = 0;
vizRate = rateControl(1 / dt);
frameSize = 0.8 * 0.5;

% Monte Carlo Localization (MCL) model
numParticles = 1000;
initialCovariance = eye(3);
mcl = monteCarloLocalization('InitialPose', initialPose, 'InitialCovariance', initialCovariance, 'ParticleLimits', [500, numParticles]);

%motion model and sensor model
mcl.MotionModel = odometryMotionModel;
sensorModel = likelihoodFieldSensorModel;
sensorModel.Map = map;
sensorModel.SensorLimits = [0.1, 10];
sensorModel.NumBeams = 10;
mcl.SensorModel = sensorModel;
mcl.GlobalLocalization = false;
mcl.UseLidarScan = true;

%arrays for storing trajectories
robotTrajectory = initialPose;

% Simulation loop
figure;
ticks = 0;
flag = 0;
recovery = 0;
i = 1;

while distToTarget > 0.2  % Run until the robot reaches the target with a threshold distance
    i = i + 1;

    % the sensor data
    [ranges, angles] = sensor(pose(:, i - 1)', map);
    [minRange, minIdx] = min(ranges);
    closestObstacleAngle = angles(minIdx);
    scan = lidarScan(ranges, angles);

    % Path-following logic
    targetDirection = atan2(target(2) - pose(2, i - 1), target(1) - pose(1, i - 1));
    headingError = targetDirection - pose(3, i - 1);
    omega = 2 * atan2(sin(headingError), cos(headingError));

    % Collision avoidance logic
    if minRange < safeDistance
        if closestObstacleAngle <= 0
            omega = 0.5;
            recovery = -0.5;
        else
            omega = -0.5;
            recovery = 0.5;
        end
        flag = 0;
        ticks = ticks + 1;
    else
        if ticks > 0
            omega = recovery;
            flag = 1;
            ticks = ticks - 1;
        elseif ticks == 0
            flag = 0;
        end
    end

    % Update robot pose
    vel = derivative(robot, pose(:, i - 1)', [v omega]);
    pose(:, i) = pose(:, i - 1) + (vel * dt);
    distToTarget = sqrt((pose(1, i) - target(1))^2 + (pose(2, i) - target(2))^2);  % Update distance to target
    robotTrajectory = [robotTrajectory, pose(:, i)]; % Append current pose to trajectory

    % Monte Carlo Localization update
    odometryPose = pose(:, i);
    [isUpdated, estimatedPose(:, i), covariance] = mcl(odometryPose, scan);

    % Visualization
    show(map);
    hold on;
    grid on;
    quiver(inpath(:,1), inpath(:,2), cos(inpath(:,3)), sin(inpath(:,3)), 0.1, 'b--'); % Input path
    quiver(optpath(:,1), optpath(:,2), cos(optpath(:,3)), sin(optpath(:,3)), 0.1, 'r-'); % Optimized path
    plot(target(1), target(2), 'gx', 'MarkerSize', 10, 'LineWidth', 2); % Target position
    plot(robotTrajectory(1, :), robotTrajectory(2, :), 'm', 'LineWidth', 1.5); % Robot trajectory
    % plot([initialPose(1), target(1)], [initialPose(2), target(2)], 'b--'); % Desired path

    T1 = [pose(1, i); pose(2, i); 0];
    T2 = axang2quat([0 0 1 pose(3, i)]);
    plotTransforms(T1', T2, "MeshFilePath", "groundvehicle.stl", "Parent", gca, "View", "2D", "FrameSize", frameSize);
    hold off;

    % Control rate
    waitfor(vizRate);
end

% Final plot with all paths
figure;
show(map);
hold on;
quiver(inpath(:,1), inpath(:,2), cos(inpath(:,3)), sin(inpath(:,3)), 0.1, 'b--'); % Input path
quiver(optpath(:,1), optpath(:,2), cos(optpath(:,3)), sin(optpath(:,3)), 0.1, 'r-'); % Optimized path
plot(target(1), target(2), 'gx', 'MarkerSize', 10, 'LineWidth', 2); % Target position
plot(robotTrajectory(1, :), robotTrajectory(2, :), 'm', 'LineWidth', 1.5); % Robot trajectory
legend("Input Path", "Optimized Path", "Target Position", "Robot Trajectory");
xlabel("X [meters]");
ylabel("Y [meters]");
title("Robot Path with Input, Optimized, and Trajectory Paths");
hold off;
