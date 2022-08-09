% RBE 501 - Robot Dynamics - Spring 2022
% Homework 3, Problem 1
% Worcester Polytechnic Institute
%
% Instructor: L. Fichera <lfichera@wpi.edu>
% Last modified: 03/22/2022
clear, clc, close all
addpath('utils');

plotOn = true; 
nTests = 10;

%% Create the manipulator
% Link length values (meters)
L1 = 0.3;
L2 = 0.3;
L3 = 0.3;

robot = SerialLink([Revolute('a', 0, 'd', L1, 'alpha', pi/2, 'offset', pi/2), ...
                    Revolute('a', L2, 'd', 0, 'alpha', 0), ...
                    Revolute('a', L3, 'd', 0, 'alpha', pi/2, 'offset', -pi/2)], ...
                    'name', 'RRR Manipulator');

% Joint limits
qlim = [-pi/2  pi/2;  % q(1)
        -pi/4  pi/2;  % q(2)
        -pi/12 pi/3]; % q(3)

% Display the manipulator in the home configuration
q = zeros(1,3);
robot.teach(q);


%% Part A - Forward Kinematics via PoE in the body frame

% Let us calculate the homogeneous transformation matrix M for the
% home configuration
R_home = [0 0 -1; 1 0 0; 0 -1 0]';
t_home = [0 L2 L1-L3]';
M = [R_home t_home; 0 0 0 1];

% Let us calculate the screw axis for each joint
% Put all the axes into a 6xn matrix S, where n is the number of joints
S_space = [0 0 1 0 0 0;
           1 0 0 -cross([1 0 0], [0 0 L1]);
           1 0 0 -cross([1 0 0], [0 L2 L1])]';
S_body = twistspace2body(S_space(:, 1), M);
for i = 2:length(S_space(1, :))
    S_body = horzcat(S_body, twistspace2body(S_space(:, i), M));
end

fprintf('---------------------Forward Kinematics Test---------------------\n');
fprintf(['Testing ' num2str(nTests) ' random configurations.\n']);
fprintf('Progress: ');
nbytes = fprintf('0%%');

% Test the forward kinematics for 10 random sets of joint variables
for ii = 1 : nTests
    fprintf(repmat('\b',1,nbytes));
    nbytes = fprintf('%0.f%%', ceil(ii/nTests*100));
    
    % Generate a random configuration
    q = [qlim(1,1) + (qlim(1,2) - qlim(1,1)) * rand(), ...
        qlim(2,1) + (qlim(2,2) - qlim(2,1)) * rand(), ...
        qlim(3,1) + (qlim(3,2) - qlim(3,1)) * rand()];
    
    % Calculate the forward kinematics
    T = fkine(S_body,M,q,"body");
    
    if plotOn
        robot.teach(q);
        title('Forward Kinematics Test');
    end
    
    assert(all(all(abs(double(robot.fkine(q)) - T) < 1e-10)));
end
 
fprintf('\nTest passed successfully.\n');


%% Part B - Calculate the Body Jacobian of the manipulator
fprintf('-------------------Differential Kinematics Test------------------\n');
fprintf(['Testing ' num2str(nTests) ' random configurations.\n']);
fprintf('Progress: ');
nbytes = fprintf('0%%'); 

% Test the correctness of the Jacobian for 10 random sets of joiny
% variables
for ii = 1 : nTests
    fprintf(repmat('\b',1,nbytes));
    nbytes = fprintf('%0.f%%', ceil(ii/nTests*100));
    
    % Generate a random configuration
    q = [qlim(1,1) + (qlim(1,2) - qlim(1,1)) * rand(), ...
        qlim(2,1) + (qlim(2,2) - qlim(2,1)) * rand(), ...
        qlim(3,1) + (qlim(3,2) - qlim(3,1)) * rand()];
    
    % Calculate the Jacobian in the body frame
    J_b = jacobe(S_space,M,q); % must use space frame for jacobian
    
    if plotOn
        robot.teach(q);
        title('Differential Kinematics Test');
    end
    
    % Test the correctness of the Jacobian
    J_test = [J_b(4:6,:); J_b(1:3,:)]; % swap the rotation and translation components
    assert(all(all(abs(double(robot.jacobe(q)) - J_test) < 1e-10)));
end

fprintf('\nTest passed successfully.\n');


% Part C - Calculate the Analyical Jacobian of the manipulator
fprintf('---------------------Analytical Jacobian Test--------------------\n');
fprintf(['Testing ' num2str(nTests) ' random configurations.\n']);
fprintf('Progress: ');
nbytes = fprintf('0%%'); 

% Test the correctness of the Analytical Jacobian for 10 random sets of joint
% variables
for ii = 1 : nTests
    fprintf(repmat('\b',1,nbytes));
    nbytes = fprintf('%0.f%%', ceil(ii/nTests*100));
    
    % Generate a random configuration
    q = [qlim(1,1) + (qlim(1,2) - qlim(1,1)) * rand(), ...
        qlim(2,1) + (qlim(2,2) - qlim(2,1)) * rand(), ...
        qlim(3,1) + (qlim(3,2) - qlim(3,1)) * rand()];
    
    % Calculate the Analytical Jacobian
    J_a = jacoba(S_space,M,q); 
    
    if plotOn
        robot.teach(q);
        title('Analytical Jacobian Test');
    end
    
    % Test the correctness of the Jacobian
    Jref = robot.jacob0(q);
    Jref = Jref(1:3,:);
    assert(all(all(abs(double(Jref) - J_a) < 1e-10)));
end

fprintf('\nTest passed successfully.\n');


%% Part D - Inverse Kinematics
nTests = 50;
fprintf('----------------------Inverse Kinematics Test--------------------\n');
fprintf(['Testing ' num2str(nTests) ' random configurations.\n']);
fprintf('Progress: ');
nbytes = fprintf('0%%');

% Set the current joint variables
currentQ = zeros(1,3);

% Calculate the Analytical Jacobian at the current configuration
J_a = jacoba(S_space,M,currentQ);

% Generate path to follow
t = linspace(0, 2*pi, nTests);
x = 0.25 * cos(t);
y = 0.25 * sin(t);
z = 0.2 * ones(1,nTests);
path = [x; y; z];

if plotOn
    robot.teach(currentQ);
    h = plot_ellipse(J_a*J_a');
    title('Inverse Kinematics Test');
    hold on
    scatter3(path(1,:), path(2,:), path(3,:), 'filled');
end

% Iterate over the target points
for ii = 1 : nTests
    fprintf(repmat('\b',1,nbytes));
    nbytes = fprintf('%0.f%%', ceil(ii/nTests*100));
    
    % Select the next target point
    targetPose = path(:,ii);
    T = fkine(S_body, M, currentQ, 'body');
    currentPose = T(1:3,4);
    
    while norm(targetPose - currentPose) > 1e-3
        J = jacoba(S_space, M, currentQ);
        % YOUR INVERSE KINEMATICS CODE HERE
        % Necessary variables:
        % Current Robot Pose -> currentPose
        % Target Robot Pose ->  targetPose
        % Current Joint Variables -> currentQ

        lambda = 0.05;
        alpha = 0.05;
%         deltaQ = gradient_inverse(J, targetPose, currentPose, alpha);
        deltaQ = damped_inverse(J, targetPose, currentPose, lambda);
%         deltaQ = vanila_inverse(J, targetPose, currentPose);
        currentQ = currentQ + deltaQ';       
        T = fkine(S_body,M,currentQ, "body");
        currentPose = T(1:3,4);
        norm(targetPose - currentPose);
    end
    robot.teach(currentQ);
    J = jacoba(S_space, M, currentQ);
    T = fkine(S_body,M,currentQ, "body");
    reset(h)
    h = plot_ellipse(J*J', T(1:3, 4));
    title('Inverse Kinematics Test');
    scatter3(path(1,:), path(2,:), path(3,:), 'filled');
end

fprintf('\nTest passed successfully.\n');
