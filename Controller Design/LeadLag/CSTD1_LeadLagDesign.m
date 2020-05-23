% #########################################################################
% TUHH :: Institute for Control Systems :: Control Lab
% #########################################################################
% Experiment CSTD1: Identification and Control of a Torsional Plant
%
% Copyright Herbert Werner and Hamburg University of Technology, 2014
% #########################################################################
% This file is to be completed by the student.
% The completed version is to be published using 
%   publish('CSTD1_LeadLagDesign','pdf') 
% and submitted as a pdf-file at least one week prior to the scheduled date
% for the experiment
%
% !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
% !!!  The gaps in the code are denoted by XXX !!!
% !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
%
% HINT 1:
% if you want to find out more about a certain command, just type 
% 'help command' into the matlab window
% HINT 2:
% use section evaluations (Ctrl+Enter) to run the code within a single 
% section

%----------------------------
% v.1.2 - 30-10-2017
% by Antonio Mendez G
%----------------------------

%%
clc
clear all
close all

%% 1 Frequency Response
%
% Calculate the transfer function of the system.

load IdentifiedSystem

sys_tf = XXX;

w_bode = logspace(-2,2,1000);

% Plot the bode diagram of the system
bode(XXX, w_bode)
grid on
hold on

%% 2 Lead Lag Design 
%
% Here, design a Lead Lag compensator, either by the procedure described
% in the lecture notes or by the loop shaping procedure. In here, few steps
% are covered for the loop shaping procedure. You are very welcome to use
% the steps described in the lecture notes as well.
%
%
% Define the bandwidth wb.
% C: lead lag compensator
wb = XXX;
C = 1; %Initial controller 

% Obtain the gain from the open loop plant that achieves the desired
% bandwith. Plug this gain in C
% hint: use evalfr()
C = C*XXX;
bode(sys_tf*C, w_bode)

% Design a lead compensator to obtain the desired phase margin
% hint: use makeweight() for lead and lag compensators.
C = C*XXX;
bode(sys_tf*C, w_bode)


% Design a roll-off filter (e.g. notch)
% to reduce the gain of the resonant peaks.
% make sure to use unity low frequency gain to not change the crossover,
C = C*XXX;
bode(sys_tf*C, w_bode)



legend('Open Loop', 'Bandwidth Adjustment', ...
    'Phase Margin Increse', 'Peaks Suppresion')
% If necessary, use SISOtool to further tune your controller, for that,
% uncomment the following line:
% sistool(sys_tf, C)

%% 3 Simulation
%
% Step response of the closed loop system
dist_on = 0; %disturbances are off

figure
sim('CSTD1_ClosedLoopLeadLag');
plot(tout, sim_reft3);
hold on
plot(tout, sim_Theta3);
title('Reference tracking (Lead Lag)');
ylabel('\Theta_3 [deg]')
xlabel('Time [s]')

grid on

% Control input
figure
plot(tout, sim_Torque);
title('Control Signal (Lead Lag)');
ylabel('Torque [Nm]')
xlabel('Time [s]')
grid on

%% 4 Frequency Responses
%
% Bode diagram of the controller
figure
bode(XXX, w_bode)
grid on
title('Controller Bode Diagram');

% Bode diagram of the closed-loop system
figure
bode(XXX, w_bode)
grid on
title('Closed Loop Bode Diagram');

% Bode diagram of the output/input disturbance rejection
% Compute the corresponding transfer functions from the output/input
% disturbances to the measured signal
figure
bode(XXX, w_bode)
grid on
hold on
bode(XXX, w_bode)
title('Output and Input Disturbance Rejection Bode Diagram');

%% 5 Disturbance Rejection
%
% Step response of the closed loop system under disturbances
%
dist_on = 1; %disturbances are on

figure
sim('CSTD1_ClosedLoopLeadLag');
plot(tout, sim_reft3);
hold on
plot(tout, sim_Theta3);
title('Disturbance Rejection (Lead Lag)');
ylabel('\Theta_3 [deg]')
xlabel('Time [s]')
grid on

%% 6 Torsional Springs Safety

% 
% The plot of the twist angle of the torsional springs
% is shown. The absolute twist value should be always 
% smaller than  20 degrees.
figure()

suptitle('All signals must be smaller than 20 degrees')

subplot(2,1,1)
plot(tout, abs(sim_Theta2-sim_Theta1));
grid on
ylabel('Twist [deg]')
legend('|\theta_2 - \theta_1|')

subplot(2,1,2)
plot(tout, abs(sim_Theta2-sim_Theta3d));
grid on
xlabel('Time [s]')
legend('|\theta_2 - \theta_3|')
ylabel('Twist [deg]')