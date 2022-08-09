function [S,M] = make_kinematics_model()
% MAKE_KINEMATICS_MODEL Calculates the Screw Axes and Home Configuration of
% a robot.
%
% Inputs: None
%
% Output: S - 6xn matrix whose columns are the screw axes of the robot
%         M - homogeneous transformation representing the home configuration

L1 = 0.4; % Lenght of Link 1 [m]
L2 = 0.2; % Lenght of Link 2 [m]
L3 = 0.2; % Lenght of Link 3 [m]
L4 = 0.17; % Lenght of Link 4 [m]
L5 = 0.17; % Lenght of Link 5 [m]
L6 = 0.126; % Lenght of Link 6 [m]

%% YOUR CODE HERE

% Screw Axes
S = [0 0 1 0 0 0; % 1
     0 -1 0 -cross([0 -1 0], [0 0 L1]); %2
     1 0 0 -cross([1 0 0], [0 0 L1]);%3
     0 -1 0 -cross([0, -1, 0], [L2+L3 0 L1]);%4
     0 0 1 -cross([0 0 1], [L2+L3 0 0]);%5
     0 -1 0 -cross([0 -1 0], [L2+L3 0 L1+L4+L5])]';%6


% Home configuration
R_home = [1 0 0; 0 0 1; 0 -1 0]';
t_home = [L2+L3+L6 0 L1+L4+L5]';
M = [R_home t_home; 0 0 0 1];
end