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
stanf

if plotOn
   stanf.teach(zeros(1,6)); 
end

%% YOUR CODE HERE