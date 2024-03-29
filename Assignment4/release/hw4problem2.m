% RBE 501 - Robot Dynamics - Spring 2022
% Homework 4, Problem 2
% Worcester Polytechnic Institute
%
% Instructor: L. Fichera <lfichera@wpi.edu>
% Last modified: 04/06/2022
clear, clc, close all
addpath('utils');

plotOn = true;

% Create the environment and the robot
g = [0 0 -9.81]; % Gravity Vector [m/s^2]
n = 6; % degrees of freedom
mdl_ur5 % Load the robot model

% Display the robot
ur5.plot(qz,'jaxes'), hold on;

% *** Define the link inertial properties ***
% Values taken from: https://www.universal-robots.com/articles/ur/application-installation/dh-parameters-for-calculations-of-kinematics-and-dynamics/
% And: http://hades.mech.northwestern.edu/images/b/b6/UR5-parameters.m
m1 = 3.7;  % [kg] Mass of Link 1
m2 = 8.4;  % [kg] Mass of Link 2
m3 = 2.33; % [kg] Mass of Link 3
m4 = 1.22; % [kg] Mass of Link 4
m5 = 1.22; % [kg] Mass of Link 5
m6 = 0.19; % [kg] Mass of Link 6

% Rotational Inertia Matrices
Ib1 = diag([0.010267495893, 0.010267495893, 0.00666]);  % Rotational Inertia Matrix of Link 1
Ib2 = diag([0.22689067591, 0.22689067591, 0.0151074]);   % Rotational Inertia Matrix of Link 2
Ib3 = diag([0.049443313556, 0.049443313556, 0.004095]);  % Rotational Inertia Matrix of Link 3
Ib4 = diag([0.111172755531, 0.111172755531, 0.21942]);   % Rotational Inertia Matrix of Link 4
Ib5 = diag([0.111172755531, 0.111172755531, 0.21942]);   % Rotational Inertia Matrix of Link 5
Ib6 = diag([0.0171364731454, 0.0171364731454, 0.033822]);% Rotational Inertia Matrix of Link 6

% Spatial Inertia Matrices
G1 =  [Ib1 zeros(3,3);
        zeros(3,3) m1*eye(3)];
G2 = [Ib2 zeros(3,3);
        zeros(3,3) m2*eye(3)];
G3 = [Ib3 zeros(3,3);
        zeros(3,3) m3*eye(3)];
G4 = [Ib4 zeros(3,3);
        zeros(3,3) m4*eye(3)];
G5 = [Ib5 zeros(3,3);
        zeros(3,3) m5*eye(3)];
G6 = [Ib6 zeros(3,3);
        zeros(3,3) m6*eye(3)];
Glist = cat(3, G1, G2, G3, G4, G5, G6);

% Load the location of the link frames
[M01, M12, M23, M34, M45, M56, M67] = calculatelinkframes(ur5);
M1 = M01;
M2 = M1 * M12;
M3 = M2 * M23;
M4 = M3 * M34;
M5 = M4 * M45;
M6 = M5 * M56;
M = M6 * M67;
Mlist = cat(3, M01, M12, M23, M34, M45, M56, M67);

% Screw Axis (space frame)
S = [0 0 1 0 0 0;
     0 -1 0 -cross([0 -1 0], [0 0 0.089459]);
     0 -1 0 -cross([0 -1 0], [-0.425 0 0.089459]);
     0 -1 0 -cross([0 -1 0], [-0.8173 0 0.089459]);
     0 0 -1 -cross([0 0 -1], [-0.8173 -0.10915 0]);
     0 -1 0 -cross([0 -1 0], [-0.8173 0 -0.0052])]';


fprintf('--------------------Dynamic Control of the UR-5 Robot-------------------\n');
% We will first generate a path in task space and solve the inverse
% kinematics for each of these points.
close all

% Initialize the matrix to store the IK result
nPts = 50;
targetQ = zeros(n,nPts);

% Set the current joint variables
currentQ = zeros(1,n);

% Generate a spiral
fprintf('Generating task space path... ');
phi = linspace(0, 4*pi, nPts);
r = linspace(0, 0.3, nPts) ;
x = r .* cos(phi) + 0.4;
y = r  .* sin(phi);
z = 0.2 * ones(1,nPts);
path = [x; y; z];
fprintf('Done.\n');

fprintf('Calculating the Inverse Kinematics... ');
ur5.plot(currentQ);
hold on
scatter3(path(1,:), path(2,:), path(3,:), 'filled');
title('Inverse Kinematics')
drawnow

% <YOUR CODE HERE>
% Iterate over the target points
for ii = 1 : nPts
    % Select the next target point
    targetPose = path(:,ii);
    T = fkine(S,M,currentQ,'space');
    currentPose = T(1:3,4);

    while norm(targetPose - currentPose) > 1e-3
        J_a = jacoba(S,M,currentQ);

        % Use the Levenberg-Marquadt algorithm (Damped Least Squares)
        lambda = 0.5;
        deltaQ = J_a' * pinv(J_a*J_a' + lambda^2 * eye(3)) * (targetPose - currentPose);

        currentQ = currentQ + deltaQ';
        T = fkine(S,M,currentQ,'space');
        currentPose = T(1:3,4);
    end

    targetQ(:,ii) = currentQ;
end

fprintf('Done.\n');

% Now, for each pair of consecutive set points, we will first calculate a
% trajectory between these two points, and then calculate the torque
% profile necessary to move from one point to the next.
fprintf('Generate the Joint Torque Profiles... ');

% Initialize the arrays where we will accumulate the output of the robot
% dynamics, so that we can display it later
qtt = []; % Joint Variables
tau = [];

for jj = 1 : nPts - 1
    t0 = 0; tf = 0.5; % Starting and ending time of each trajectory
    N = 500;          % Number of intermediate setpoints
    t = linspace(t0, tf, N); % time vector

    q = zeros(n,N);   % joint variables
    qd = zeros(n,N);  % joint velocities
    qdd = zeros(n,N); % joint accelerations

    for ii = 1 : n
        % Calculate the coefficients of the quintic polynomial
        a = quinticpoly(t0, tf, ...
            targetQ(ii,jj), targetQ(ii,jj+1), ...
            0, 0, 0, 0);

        % Generate the joint profiles (position, velocity, and
        % acceleration)
        q(ii,:) = a(1) + a(2) * t + a(3) * t.^2 + a(4) * t.^3 + a(5) * t.^4 + a(6) * t.^5;
        qd(ii,:) = a(2) + 2*a(3)*t + 3*a(4)*t.^2 + 4*a(5)*t.^3 + 5*a(6)*t.^4;
        qdd(ii,:) = 2*a(3) + 6*a(4)*t + 12*a(5)*t.^2 + 20*a(6)*t.^3;
    end

    % Use the equations of motion to calculate the necessary torques to trace
    % the trajectory
    Ftipmat = zeros(N,6); % no end effector force
    taumat = InverseDynamicsTrajectory(q', qd', qdd', ...
        g, Ftipmat, Mlist, Glist, S);

    % Use the Forward Dynamics to simulate the robot behavior
    dt = tf/N;  % time step
    intRes = 1; % Euler integration constant
    [qt, qdt] = ForwardDynamicsTrajectory(q(:,1), qd(:,1), taumat, g, ...
        Ftipmat, Mlist, Glist, S, dt, ...
        intRes);

    qtt = [qtt; qt]; % Accumulate the results
    tau = [tau; taumat];
end

fprintf('Done.\n');

fprintf('Simulate the robot...');
title('Inverse Dynamics Control');
ur5.plot(qtt(1:10:end,:));
fprintf('Done.\n');

% Display the Joint Torques
figure, hold on, grid on
plot((1:length(tau))*dt, tau(:,1), 'Linewidth', 2);
plot((1:length(tau))*dt, tau(:,2), 'Linewidth', 2);
plot((1:length(tau))*dt, tau(:,3), 'Linewidth', 2);
plot((1:length(tau))*dt, tau(:,4), 'Linewidth', 2);
plot((1:length(tau))*dt, tau(:,5), 'Linewidth', 2);
plot((1:length(tau))*dt, tau(:,6), 'Linewidth', 2);
xlim([0 max((1:length(tau))*dt)]);
xlabel('Time [s]'), ylabel('Torque [Nm]');
legend({'Joint 1', 'Joint 2', 'Joint 3','Joint 4', 'Joint 5', 'Joint 6'});
set(gca, 'FontSize', 14);

fprintf('Program completed successfully.\n');
