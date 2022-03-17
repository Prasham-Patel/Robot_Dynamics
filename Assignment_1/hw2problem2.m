% RBE 501 - Robot Dynamics - Spring 2022
% Homework 2, Problem 2
% Worcester Polytechnic Institute
%
% Instructor: L. Fichera <lfichera@wpi.edu>
% Last modified: 02/06/2022
clear, clc, close all
addpath('utils');

plotOn = false;
nTests = 20; % number of random test configurations

%% Create the manipulator
% robot length values (meters)
L0 = 0.3;
L1 = 0.3;
L2 = 0.3;

robot = SerialLink([Revolute('a', 0, 'd', L0, 'alpha', -pi/2, 'offset', pi/2), ...
                    Revolute('a', L1, 'd', 0, 'alpha', 0), ...
                    Revolute('a', L2, 'd', 0, 'alpha', pi/2), ...
                    Revolute('a', 0, 'd', 0, 'alpha', -pi/2), ...
                    Revolute('a', 0, 'd', 0, 'alpha', -pi/2, 'offset', -pi/2), ...
                    Revolute('a', 0, 'd', 0, 'alpha', 0)], 'name', 'Elbow Manipulator'); 

% Joint limits
qlim = [-pi/2  pi/2;  % q(1)
        -pi/4  pi/2;  % q(2)
        0      pi/3;  % q(3)
        -pi/2  pi/2;  % q(4)
        -pi/2  pi/2;  % q(5)
        -pi/2  pi/2]; % q(6)

% Display the manipulator in the home configuration
q = zeros(1,6);
robot.teach(q);

% screw axis defination

w1 = [0, 0, 1];
p1 = [0, 0, L0]';
v1 = -skew(w1)*p1;
S = [w1, v1']';

w2 = [-1, 0, 0];
p2 = [0, 0, L0]';
v2 = -skew(w2)*p2;
S = horzcat(S, [w2, v2']');

w3 = [-1, 0, 0];
p3 = [0, L1, L0]';
v3 = -skew(w3)*p3;
S = horzcat(S, [w3, v3']');

w4 = [0, 0, 1];
p4 = [0, L1 + L2, L0]';
v4 = -skew(w4)*p4;
S = horzcat(S, [w4, v4']');

w5 = [-1, 0, 0];
p5 = [0, L1 + L2, L0]';
v5 = -skew(w5)*p5;
S = horzcat(S, [w5, v5']');

w6 = [0, 1, 0];
p6 = [0, L1 + L2, L0]';
v6 = -skew(w6)*p6;
S = horzcat(S, [w6, v6']');

% home matrix defination

R = [0, 0, 1; 1, 0, 0; 0 , 1, 0]';
P = [0, L1 + L2, L0]';
M = [R, P; 0, 0, 0, 1];

% forward kinematics testing
    
T = fkine(S, M, q);
fprintf('---------------------Forward Kinematics Test---------------------\n');
fprintf(['Testing ' num2str(nTests) ' random configurations.\n']);
fprintf('Progress: ');
nbytes = fprintf('0%%'); 
 
% Test the forward kinematics for 100 random sets of joint variables
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
    T = fkine(S, M, q);
    
    if plotOn
        robot.teach(q);
        title('Forward Kinematics Test');
    end
    robot.fkine(q)
    assert(all(all(abs(double(robot.fkine(q)) - T) < 1e-10)));
end
 
fprintf('\nTest passed successfully.\n');


%% Part C - Calculate the Space Jacobian of the manipulator
fprintf('-------------------Differential Kinematics Test------------------\n');
fprintf(['Testing ' num2str(nTests) ' random configurations.\n']);
fprintf('Progress: ');
nbytes = fprintf('0%%'); 

% Test the correctness of the Jacobian for 100 random sets of joint
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
    
    % Calculate the Forward Kinematics
    T = fkine(S,M,q);
    
    % Calculate the Jacobian
    J = jacob0(S, q);
    
    if plotOn
        robot.teach(q);
        title('Differential Kinematics Test');
    end
    
    % Test the correctness of the Jacobian
    Jcoords = [-skew(T(1:3,4))*J(1:3,:)+J(4:6,:); J(1:3,:)];
    assert(all(all(abs(double(robot.jacob0(q)) - Jcoords) < 1e-10)));
end

fprintf('\nTest passed successfully.\n');

%% Part D - Inverse Kinematics
fprintf('----------------------Inverse Kinematics Test--------------------\n');
fprintf(['Testing ' num2str(nTests) ' random configurations.\n']);
fprintf('Progress: ');
nbytes = fprintf('0%%');

% plotOn = true;

% Calculate the twist representing the robot's home pose
currentPose = MatrixLog6(M);
currentPose = [currentPose(3,2) currentPose(1,3) currentPose(2,1) currentPose(1:3,4)']';

% Set the current joint variables
currentQ = zeros(1,6);

if plotOn
    robot.teach(currentQ);
    h = triad('matrix', M, 'tag', 'Target Pose', 'linewidth', 2.5, 'scale', 0.5);
end

% % Joint limits
% qlim = [-pi/2  pi/2;  % q(1)
%         -pi/4  pi/2;  % q(2)
%         0      pi/3;  % q(3)
%         -pi/2  pi/2;  % q(4)
%         -pi/2  pi/2;  % q(5)
%         -pi/2  pi/2]; % q(6)

% Generate the test configurations
% q = [linspace(0,pi/3,nTests);
%      linspace(0,pi/3,nTests);
%      linspace(pi/36,pi/8,nTests);
%      linspace(0,pi/3,nTests);
%      linspace(0,pi/3,nTests);
%      linspace(0,pi/3,nTests)];

% Generate the test configurations [TEST]
q = [linspace(0,pi/6,nTests);
     linspace(0,pi/6,nTests);
     linspace(0,pi/6,nTests);
     linspace(0,pi/6,nTests);
     linspace(0,pi/6,nTests);
     linspace(0,pi/6,nTests)];

for ii = 1 : nTests
    fprintf(repmat('\b',1,nbytes));
    nbytes = fprintf('\n %0.f%%', ceil(ii/nTests*100));
    
    % Generate the robot's pose
    T = fkine(S,M,q(:,ii)');
    targetPose = MatrixLog6(T);
    targetPose = [targetPose(3,2) targetPose(1,3) targetPose(2,1) targetPose(1:3,4)']'
    
    if plotOn
        set(h, 'matrix', T);
        title('Inverse Kinematics Test');
        drawnow;
    end
    
    % Inverse Kinematics
    fprintf("done \n")
    while norm(targetPose - currentPose) > 1e-3
        J = jacob0(S, currentQ);
        nor = norm(targetPose - currentPose);

%         lambda = 0.002*(nor - 0.0009)^-1
%         alpha = (nor)*10;
        
        lambda = 4;
        alpha = 0.05;

        if ( norm(targetPose - currentPose) > 0.01)
            deltaQ = gradient_inverse(J, targetPose, currentPose, alpha);
        else
            deltaQ = damped_inverse(J, targetPose, currentPose, lambda);
        end
%         deltaQ = vanila_inverse(J, targetPose, currentPose);
        currentQ = currentQ + deltaQ';
       
        T = fkine(S,M,currentQ);
        currentPose = MatrixLog6(T);
        currentPose = [currentPose(3,2) ...
                       currentPose(1,3) ...
                       currentPose(2,1) ...
                       currentPose(1:3,4)']';
%         fprintf("stuck \n")
        if plotOn
            try
                robot.teach(currentQ);
                drawnow;
            catch e
                continue;
            end
        end
    end
end

fprintf('\nTest passed successfully.\n');


