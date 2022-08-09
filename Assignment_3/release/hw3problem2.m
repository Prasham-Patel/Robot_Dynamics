% RBE 501 - Robot Dynamics - Spring 2022
% Homework 3, Problem 2
% Worcester Polytechnic Institute
%
% Instructor: L. Fichera <lfichera@wpi.edu>
% Last modified: 03/22/2022
clear, clc, close all
addpath('utils');

plotOn = true; 
nTests = 10;

%% Create the manipulator
mdl_stanford
robot = stanf;
qlim = robot.qlim

if plotOn
   robot.teach(zeros(1,6)); 
end

%% YOUR CODE HERE

%% Part A - Forward Kinematics via PoE in the body frame

% Let us calculate the homogeneous transformation matrix M for the

% Let us calculate the homogeneous transformation matrix M for the
% home configuration
R_h = [0, -1, 0; 1, 0, 0; 0, 0, 1]';
P_h = [0, 0.154, 0.675,]';
M = [R_h, P_h; 0, 0, 0, 1];

% Let us calculate the screw axis for each joint
% Put all the axes into a 6xn matrix S, where n is the number of joints

S_space = [0 0 1 0 0 0;
           0 1 0 -cross([0 1 0], [0 0 0.412]);
           0 0 0 0 0 1;
           0 0 1 -cross([0 0 1], [0 0.154 0]);
           1 0 0 -cross([1 0 0], [0 0.154 0.412]);
           0 0 1 -cross([0 0 1], [0 0.154 0])]';
 
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
        qlim(3,1) + (qlim(3,2) - qlim(3,1)) * rand(), ...
        qlim(4,1) + (qlim(4,2) - qlim(4,1)) * rand(), ...
        qlim(5,1) + (qlim(5,2) - qlim(5,1)) * rand(), ...
        qlim(6,1) + (qlim(6,2) - qlim(6,1)) * rand()];
    
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
        qlim(3,1) + (qlim(3,2) - qlim(3,1)) * rand(), ...
        qlim(4,1) + (qlim(4,2) - qlim(4,1)) * rand(), ...
        qlim(5,1) + (qlim(5,2) - qlim(5,1)) * rand(), ...
        qlim(6,1) + (qlim(6,2) - qlim(6,1)) * rand()];
        
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
        qlim(3,1) + (qlim(3,2) - qlim(3,1)) * rand(), ...
        qlim(4,1) + (qlim(4,2) - qlim(4,1)) * rand(), ...
        qlim(5,1) + (qlim(5,2) - qlim(5,1)) * rand(), ...
        qlim(6,1) + (qlim(6,2) - qlim(6,1)) * rand()];
    
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
currentQ = [0, 0, 0, 0, 0, 0];

% Calculate the Analytical Jacobian at the current configuration
J_a = jacoba(S_space,M,currentQ);

% Generate path to follow
t = linspace(0, 2*pi, nTests);
x = 0.5 * cos(t);
y = 0.5 * sin(t);
z = 0.6 * ones(1,nTests);
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
        currentQ = currentQ + deltaQ'   ;  
        T = fkine(S_body,M,currentQ, "body");
        currentPose = T(1:3,4);
        norm(targetPose - currentPose);
    end
    robot.teach(currentQ);
    J = jacoba(S_space, M, currentQ);
    T = fkine(S_body,M,currentQ, "body");
    delete(h);
    h = plot_ellipse(J*J', T(1:3, 4));
    title('Inverse Kinematics Test');
    scatter3(path(1,:), path(2,:), path(3,:), 'filled');
end

fprintf('\nTest passed successfully.\n');
