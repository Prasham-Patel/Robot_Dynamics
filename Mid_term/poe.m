% RBE 501 - Robot Dynamics - Sping 2022
% Midterm Exam
% Worcester Polytechnic Institute
%
% Student: ***Prasham Patel***
% WPI_ID = 871563809

clear, clc, close all
addpath('utils');

%% *** ENTER THE LAST DIGIT OF YOUR WPI ID BELOW: ***
digit = 9;

%% Create the manipulator
robot = make_robot(digit);
n = robot.n;
qlim = robot.qlim;
robot.teach(zeros(1,6))


%% Calculate the forward kinematics using the Product of Exponentials
% Let us calculate the screw axis for each joint
% Put all the axes into a 6xn matrix S, where n is the number of joints

l0 = 0.1348;
l1 = 0.2738;
l2 = 0.230;
l3 = 0.1283;
l4 = 0.116;
l5 = 0.105;

% screw axis defination

w1 = [0, 0, 1];
p1 = [0, 0, 0]';
v1 = -skew(w1)*p1;
S = [w1, v1']';

w2 = [1, 0, 0];
p2 = [0, 0, l0]';
v2 = -skew(w2)*p2;
S = horzcat(S, [w2, v2']');

w3 = [-1, 0, 0];
p3 = [0, 0, l0 + l1]';
v3 = -skew(w3)*p3;
S = horzcat(S, [w3, v3']');

w4 = [1, 0, 0];
p4 = [0, 0, l0 + l1 + l2]';
v4 = -skew(w4)*p4;
S = horzcat(S, [w4, v4']');

w5 = [0, 0, -1];
p5 = [l3, 0, l0 + l1 + l2]';
v5 = -skew(w5)*p5;
S = horzcat(S, [w5, v5']');

w6 = [1, 0, 0];
p6 = [l3 + l5, 0, l0 + l1 + l2 + l4]';
v6 = -skew(w6)*p6;
S = horzcat(S, [w6, v6']')

% Let us also calculate the homogeneous transformation matrix M for the
% home configuration

R = [0, -1, 0; 0, 0, -1; 1 , 0, 0]';
P = [l3 + l5, 0, l0 + l1 + l2 + l4]'; % converting from mm to meter
M = [R, P; 0, 0, 0, 1]

% % Joint limits
% qlim = [-pi/2  pi/2;  % q(1)
%         -pi/4  pi/2;  % q(2)
%         0      pi/3;  % q(3)
%         -pi/2  pi/2;  % q(4)
%         -pi/2  pi/2;  % q(5)
%         -pi/2  pi/2]; % q(6)
% 
% % forward kinematics testing
% nTests = 20
% % T = fkine(S, M, q);
% fprintf('---------------------Forward Kinematics Test---------------------\n');
% fprintf(['Testing ' num2str(nTests) ' random configurations.\n']);
% fprintf('Progress: ');
% nbytes = fprintf('0%%'); 
%  
% % Test the forward kinematics for 100 random sets of joint variables
% for ii = 1 : nTests
%     fprintf(repmat('\b',1,nbytes));
%     nbytes = fprintf('%0.f%%', ceil(ii/nTests*100));
%     
%     % Generate a random configuration
%     q = [qlim(1,1) + (qlim(1,2) - qlim(1,1)) * rand(), ...
%          qlim(2,1) + (qlim(2,2) - qlim(2,1)) * rand(), ...
%          qlim(3,1) + (qlim(3,2) - qlim(3,1)) * rand(), ...
%          qlim(4,1) + (qlim(4,2) - qlim(4,1)) * rand(), ...
%          qlim(5,1) + (qlim(5,2) - qlim(5,1)) * rand(), ...
%          qlim(6,1) + (qlim(6,2) - qlim(6,1)) * rand()];
%     
%     % Calculate the forward kinematics
%     T = fkine(S, M, q);
%     
%     
%     robot.fkine(q)
%     assert(all(all(abs(double(robot.fkine(q)) - T) < 1e-10)));
% end
%  
% fprintf('\nTest passed successfully.\n');
% 
% 
% %% Part C - Calculate the Space Jacobian of the manipulator
% fprintf('-------------------Differential Kinematics Test------------------\n');
% fprintf(['Testing ' num2str(nTests) ' random configurations.\n']);
% fprintf('Progress: ');
% nbytes = fprintf('0%%'); 
% 
% % Test the correctness of the Jacobian for 100 random sets of joint
% % variables
% for ii = 1 : nTests
%     fprintf(repmat('\b',1,nbytes));
%     nbytes = fprintf('%0.f%%', ceil(ii/nTests*100));
%     
%     % Generate a random configuration
%     q = [qlim(1,1) + (qlim(1,2) - qlim(1,1)) * rand(), ...
%          qlim(2,1) + (qlim(2,2) - qlim(2,1)) * rand(), ...
%          qlim(3,1) + (qlim(3,2) - qlim(3,1)) * rand(), ...
%          qlim(4,1) + (qlim(4,2) - qlim(4,1)) * rand(), ...
%          qlim(5,1) + (qlim(5,2) - qlim(5,1)) * rand(), ...
%          qlim(6,1) + (qlim(6,2) - qlim(6,1)) * rand()];
%     
%     % Calculate the Forward Kinematics
%     T = fkine(S,M,q);
%     
%     % Calculate the Jacobian
%     J = jacob0(S, q);
%     
%    
%     
%     % Test the correctness of the Jacobian
%     Jcoords = [-skew(T(1:3,4))*J(1:3,:)+J(4:6,:); J(1:3,:)];
%     assert(all(all(abs(double(robot.jacob0(q)) - Jcoords) < 1e-10)));
% end
% 
% fprintf('\nTest passed successfully.\n');
% 
% 
