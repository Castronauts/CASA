%%%
% RoboticArmTorqueAnalysis.m
% Author: Ben Mellinkoff
% This script will be used to calculate the torques experienced at each
% joint
%%%
%% house keeping
clear all; close all; clc;
%% givens
g= 9.81; % acceleration due to gravity [N/m^2]
% masses
m_Gird5= 1.3; % mass of 5 in girder [oz]
m_Gird2= 0.8; % mass of 2.5 in girder [oz]
m_SAServo= 3.4; % mass of single-axis servo [oz]
m_Wrist= 3.5; % mass of WristRotateAX [oz]
m_Gripper= 6.6; % mass of AX-12 gripper [oz]
m_Plate= 0.26; % mass of single-axis mount [oz]
mass= [m_Gird5;m_Gird2;m_SAServo;m_Wrist;m_Gripper;m_Plate]; % put masses into a vector
mass= mass*0.0283495; % convert units of masses from ounces to kilogram [kg]
m_Payload= 1; % mass of payload in gripper [kg]
mass= [mass;m_Payload]; % add payload mass to mass vector

% lengths to center of masses
l_Gird5= 2.5; % length to CoM of 5 in girder [in]
l_Mount1= l_Gird5+2.5; % length to CoM of first mount [in]
l_SAServo1= l_Mount1+1.455; % length to CoM of first single-axis servo [in]
l_Mount2= l_SAServo1+1.455; % length to CoM of second mount [in]
l_Gird2= l_Mount2+1.25; % length to CoM of 2.5 in girder [in]
l_Mount3= l_Gird2+1.25; % length to CoM of third mount [in]
l_SAServo2= l_Mount3+1.455; % length to CoM of second single-axis servo [in]
l_Wrist= l_SAServo2+2.91; % length of CoM of wrist-rotator servo (approx.) [in]
l_Gripper= l_Wrist+3.69375; % length of CoM of gripper (approx.) [in]
l_Payload= l_Gripper+3.73125; % length of CoM of payload (approx.) [in]
length= [l_Gird5;l_Mount1;l_SAServo1;l_Mount2;l_Gird2;l_Mount3;l_SAServo2;l_Wrist;l_Gripper;l_Payload]; % put lengths into a vector
length= length*0.0254; % convert units of length from inches to meters [m]

%% calculate torque at double-axis servo (horizontal arm -> highest torque)
Torque= g*(length(1)*mass(1)+length(2)*mass(6)+length(3)*mass(3)+length(4)*mass(6)+length(5)*mass(2)...
        +length(6)*mass(6)+length(7)*mass(3)+length(8)*mass(4)+length(9)*mass(5)+length(10)*mass(7));
% this is the torque calculated at the double-axis servo (assuming the arm is horizontal) [N*m]
Torque_FoS= Torque*1.2; % required torque with a 20% factor of safety included
