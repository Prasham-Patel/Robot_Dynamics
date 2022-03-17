% RBE 501 - Robot Dynamics - Spring 2022
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

poe

%% Calculate the Jacobian matrix in the home configuration
q = zeros(1,6);
J = jacob0(S, q);