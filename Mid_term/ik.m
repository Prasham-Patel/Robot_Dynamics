% RBE 501 - Robot Dynamics - Fall 2021
% Midterm Exam
% Worcester Polytechnic Institute
%
% Student: ***Prasham Patel***
% WPI_ID = 871563809
clear, clc, close all
addpath('utils');

% First, execute poe.m to load the S and M matrices
poe
close all

% Generate and display the path that the robot has to trace
t = linspace(-pi, pi, 36);
x = 0.3  * ones(1,36);
%a = 0.4;
y = (10 * (sin(t)).^3)./60;
z = (13 * cos(t) - 5 * cos(2*t) - 2 * cos(3*t) - cos(4*t))./60 + 0.3;
path = [x; y; z];

scatter3(path(1,:), path(2,:), path(3,:), 'filled');


% Convert Cartesian coordinates into twists
targetPose = zeros(6,size(path,2)); % each column of this matrix is a target pose represented by a twist

for ii = 1 : size(path,2)
    % First calculate the homogeneous transformation matrix representing
    % each target pose
    R = [0 0 -1; 0 1 0; 1 0 0]';
    T = [R path(:,ii); 
         0 0 0 1];
     
    % Then perform the matrix logarithm operation to convert transformation
    % matrices into 4x4 elements of se(3)
    t = MatrixLog6(T);
    
    % Finally, "unpack" this matrix (i.e., perform the inverse of the
    % bracket operator)
    targetPose(:,ii) = [t(3,2) t(1,3) t(2,1) t(1:3,4)']';
end

% Calculate the twist representing the robot's home pose
currentPose = MatrixLog6(M);
currentPose = [currentPose(3,2) currentPose(1,3) currentPose(2,1) currentPose(1:3,4)']';

%% Calculate the inverse kinematics 
currentQ = zeros(1, 6);
qList = zeros(36, 6);

for i = 1 : size(path,2)
    i

while norm(targetPose(:, i) - currentPose) >= 1e-3 % resolution less than of 1 mm
        J = jacob0(S, currentQ);
        
        lambda = 0.0; % basically vanilla inverse
        alpha = 0.05;

        if ( norm(targetPose(:, i) - currentPose) > 0.02) % if error more than 2mm use gradient inverse
            deltaQ = gradient_inverse(J, targetPose(:, i), currentPose, alpha);
        else % use damped inverse if error < 2mm
            deltaQ = damped_inverse(J, targetPose(:, i), currentPose, lambda);
        end
%         deltaQ = vanila_inverse(J, targetPose, currentPose);
        currentQ = currentQ + deltaQ';
         
        T = fkine(S,M,currentQ);
        currentPose = MatrixLog6(T);
        currentPose = [currentPose(3,2) ...
                       currentPose(1,3) ...
                       currentPose(2,1) ...
                       currentPose(1:3,4)']';
end
norm(targetPose(:, i) - currentPose)
qList(i, :) = currentQ;
end
robot.plot(qList, 'trail', {'r', 'LineWidth', 5});