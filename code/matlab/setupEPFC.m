% This setup file is based on the setup file of ARDrone plugoin

%%
%  Cleaning workspace
bdclose all;
clear all;
clc

%%
% Adding ARDrone library path 
addpath ../lib; 
%% Simulation parameters

% Flight management system sample time. This is the sample time at which
% the control law is executed. 
FMS.Ts = 0.065; 

% Time delay due to communication between drone and host computer
timeDelay = FMS.Ts*4; 


%% Vehicle model based on linear dynamics

% Loading state space representation of vehicle dynamics
setupARModel; 



%% 
% Simulation time
simDT = 0.005 ;

%%
% Loading Simulink model of ARDrone and EPFC
ARDroneEPFC ;



