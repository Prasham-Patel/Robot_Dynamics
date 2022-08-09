function [Mlist,Glist] = make_dynamics_model()
% MAKE_KINEMATICS_MODEL Creates the dynamics model of the robot
%
% Inputs: None
%
% Output: Mlist - 4x4x7 matrix containing all the transformation matrices between consecutive link frames
%         Glist - 6x6x6 matrix containing the spatial inertia matrices of each link


%% Define the link lengths and masses
L1 = 0.4; % Lenght of Link 1 [m]
L2 = 0.2; % Lenght of Link 2 [m]
L3 = 0.2; % Lenght of Link 3 [m]
L4 = 0.17; % Lenght of Link 4 [m]
L5 = 0.17; % Lenght of Link 5 [m]
L6 = 0.126; % Lenght of Link 6 [m]

m1 = 3;   % Mass of Link 1 [kg]
m2 = 0.5;   % Mass of Link 2 [kg]
m3 = 0.5;   % Mass of Link 3 [kg]
m4 = 0.5;   % Mass of Link 4 [kg]
m5 = 0.5;   % Mass of Link 5 [kg]
m6 = 0.4;   % Mass of Link 6 [kg]

r = 2.5e-2; % Link radius [m]

Ib1 = diag([m1*(3*r^2 + L1^2)/12, m1*(3*r^2 + L1^2)/12, m1*(r^2)/2 ]);
Ib2 = diag([m2*(3*r^2 + L2^2)/12, m2*(3*r^2 + L2^2)/12, m2*(r^2)/2 ]);
Ib3 = diag([m3*(3*r^2 + L3^2)/12, m3*(3*r^2 + L3^2)/12, m3*(r^2)/2 ]);
Ib4 = diag([m4*(3*r^2 + L4^2)/12, m4*(3*r^2 + L4^2)/12, m4*(r^2)/2 ]);
Ib5 = diag([m5*(3*r^2 + L5^2)/12, m5*(3*r^2 + L5^2)/12, m5*(r^2)/2 ]);
Ib6 = diag([m6*(3*r^2 + L6^2)/12, m6*(3*r^2 + L6^2)/12, m6*(r^2)/2 ])

% Spatial Inertia Matrices
G1 = [Ib1, zeros(3,3); zeros(3,3), m1*eye(3)];
G2 = [Ib2, zeros(3,3); zeros(3,3), m2*eye(3)];
G3 = [Ib3, zeros(3,3); zeros(3,3), m3*eye(3)];
G4 = [Ib4, zeros(3,3); zeros(3,3), m4*eye(3)];
G5 = [Ib5, zeros(3,3); zeros(3,3), m5*eye(3)];
G6 = [Ib6, zeros(3,3); zeros(3,3), m6*eye(3)];
Glist = cat(3, G1, G2, G3, G4, G5, G6)

R_01 = [1 0 0; 0 1 0; 0 0 1]';
t_01 = [0 0 L1/2]';
M01 = [R_01 t_01; 0 0 0 1];

R_12 = [0 0 -1; 0 1 0; 1 0 0]';
t_12 = [L2/2 0 L1/2]';
M12 = [R_12 t_12; 0 0 0 1];

R_23 = [1 0 0; 0 1 0; 0 0 1]';
t_23 = [(L2+L3)/2 0 0]';
M23 = [R_23 t_23; 0 0 0 1];

R_34 = [0 0 1; 0 1 0; -1 0 0]';
t_34 = [L3/2 0 L4/2]';
M34 = [R_34 t_34; 0 0 0 1];

R_45 = [1 0 0; 0 1 0; 0 0 1]';
t_45 = [0 0 (L4+L5/2)]';
M45 = [R_45 t_45; 0 0 0 1];

R_56 = [0 0 -1; 0 1 0; 1 0 0]';
t_56 = [L6/2 0 L5/2]';
M56 = [R_56 t_56; 0 0 0 1];

R_67 = [0 0 1; -1 0 0; 0 -1 0]';
t_67 = [L6/2 0 0]';
M67 = [R_67 t_67; 0 0 0 1];

Mlist = cat(3, M01, M12, M23, M34, M45, M56, M67)

end