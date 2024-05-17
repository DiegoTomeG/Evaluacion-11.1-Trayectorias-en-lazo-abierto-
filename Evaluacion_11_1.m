%% Evaluación 11.1 (TRAYECTORIAS LAZO ABIERTO) ---- LANDMARKS
%José Diego Tomé Guardado A01733345

%% EXAMPLE: Differential drive vehicle following waypoints using the 
% Pure Pursuit algorithm
%
% Copyright 2018-2019 The MathWorks, Inc.

%% Define Vehicle
R = 0.1;                % Wheel radius [m]
L = 0.5;                % Wheelbase [m]
dd = DifferentialDrive(R,L);

%% Simulation parameters
sampleTime = 0.05;               % Sample time [s]
tVec = 0:sampleTime:185;         % Time array

%CAMBIAR CONFORME A LAS ESPECIFICACIONES DE CADA TRAYECTORIA O DIBUJO
initPose = [2;3;2*pi];             % Initial pose (x y theta)
pose = zeros(3,numel(tVec));    % Pose matrix
pose(:,1) = initPose;

% Define waypoints
% ----- Ejercicio Trayectoria: JOSE_DIEGO -------------
waypoints = [2,3; 2,0; 1,0; 2.5,0;
             2.5,3; 3.5,3; 3.5,0; 2.5,0;
             4,0; 5,0; 5,1.5; 4,1.5; 4,3; 5,3;
             5.5,3; 6.5,3; 5.5,3; 5.5,1.5; 6.5,1.5; 5.5,1.5; 5.5,0; 6.5,0
             8,0; 8,3; 9,1.5; 8,0; 
             9.5,0; 9.5,3; 10,3; 11,3; 10,3; 10,1.5; 11,1.5; 10,1.5; 10,0; 11,0;
             11.5,0; 11.5,3; 12.5,3; 11.5,3; 11.5,0; 12.5,0; 12.5,1.5; 12,1.5; 12.5,1.5;
             12.5,0; 13,0; 14,0; 14,3; 13,3; 13,0]

% Create visualizer
viz = Visualizer2D;
viz.hasWaypoints = true;

%% Pure Pursuit Controller
controller = controllerPurePursuit;
controller.Waypoints = waypoints;
%muy largo no sabremos a que way point va a ir
controller.LookaheadDistance = 0.25; %muy corto no vera el waypoint
controller.DesiredLinearVelocity = 0.4;
controller.MaxAngularVelocity = 10;

%% Simulation loop
close all
r = rateControl(1/sampleTime);
for idx = 2:numel(tVec) 
    % Run the Pure Pursuit controller and convert output to wheel speeds
    [vRef,wRef] = controller(pose(:,idx-1));
    [wL,wR] = inverseKinematics(dd,vRef,wRef)
 
    
    % Compute the velocities
    [v,w] = forwardKinematics(dd,wL,wR);
    velB = [v;0;w]; % Body velocities [vx;vy;w]
    vel = bodyToWorld(velB,pose(:,idx-1));  % Convert from body to world
    
    % Perform forward discrete integration step
    pose(:,idx) = pose(:,idx-1) + vel*sampleTime; 
    
    % Update visualization
    viz(pose(:,idx),waypoints)
    waitfor(r);
    
end
