% #########################################################################
% TUHH :: Institute for Control Systems :: Control Lab
% #########################################################################
% Experiment CSTD2: Magnetic Levitation Plant
%
% Copyright Herbert Werner and Hamburg University of Technology, 2014
% #########################################################################
% This file is to be completed by the student.
% The completed version is to be published using 
%   publish('cstd2_design.m','pdf') 
% and submitted as a pdf-file at least one week prior to the scheduled date
% for the experiment
%
% !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
% !!!  The gaps in the code are denoted by TODO !!!
% !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
%
% HINT 1:
% if you want to find out more about a certain command, just type 
% 'help command' into the matlab window
% HINT 2:
% use section evaluations (Ctrl+Enter) to run the code within a single 
% section

%----------------------------
% v.0.9 - 13-11-2014
% by Michael Heuer 
%----------------------------
% Last modified on 25-11-2014
% by Julian Theis
% ---------------------------

%%
clear all; clc; close all

%% I. Load and scale the plant
% In the first step we load the plant which was identified in the previous task.
% After that the system matrices are extrected and the number of state,
% inputs and outputs are stored, cause we need them later.

load d_steps.mat

% Choose the model for the controller design
sys = ss(A,B,C,D);

% Extract the relevant matrices
[A,B,C,D] = ssdata(sys);

% Extract the system dimensions
n  = size(A,1);
ni = size(B,2);
no = size(C,1);

%% I.b Design of a Prefilter for Reference Tracking 
%

V = 0.1*(360/293.99);

%% I.c Simulation of the Feed Forward Design
%

sys = ss1;
sim('cstd2_sim_ff');

figure(1);
t = data(:,1);
plot(t, data(:,2), t, data(:,3), t, data(:,4), t, data(:,5));
legend({'r_1','r_2','y_1','y_2'});
grid('on');

% Simulation with an other plant

sys = ss1;
sim('cstd2_sim_ff');

figure(2);
t = data(:,1);
plot(t, data(:,2), t, data(:,3), t, data(:,4), t, data(:,5));
legend({'r_1','r_2','y_1','y_2'});
grid('on');

%% II.a Design of the observer
% For the linear quadratic regulator, it is important to have access to the
% states, which are not measured in general. For that reason we have to
% estimate them using an Luenberg observer.

Q_obsv = diag([0 1 0 1 0 1]);
R_obsv = 0.0001;

L = -lqr(A',C3', Q_obsv, R_obsv)'; 

% Build observer system
A_obsv = A;
B_obsv = B;
C_obsv = C;
D_obsv = D;

%% II.b Analysis of the observer 

disp('Eigenvalues of the observer are: ');
damp(A_obsv);

%% III.a Design of the controller
% In the next step the optimal state feeback gains are calculated.
%

Q = C'*C;
R = 200;

F = -lqr(A, B, Q_lqr, R_lqr);

% Calculate Prefilter for Reference Tracking

V = 0.1*(360/293.99);

sys_cl = ss1;

%% III.b Analysis of the observer 
%

disp('Eigenvalues of the closed loop are: ');
damp(sys_cl);

%% III.c Simulation
%

sys = ss1;
sim('cstd2_sim_lqg');

figure(3);
t = data(:,1);
plot(t, data(:,2), t, data(:,3), t, data(:,4), t, data(:,5));
legend({'r_1','r_2','y_1','y_2'});
grid('on');

% Simulation with an other plant

sys = ss1;
sim('cstd2_sim_lqg');

figure(4);
t = data(:,1);
plot(t, data(:,2), t, data(:,3), t, data(:,4), t, data(:,5));
legend({'r_1','r_2','y_1','y_2'});
grid('on');

%% IV.a Design of the Controller with integral action
% The problem of the previous design is the steady controll offset.
% To cope that it is important to add an integrator to the controller.
%

% Build augmented plant
A_aug = [A, zeros(6, 1); -C3, 0];
B_aug = [B; 0];
C_aug = C;
D_aug = D;

% Tuning Parameter
Q_C = C'*C;

Q_aug = [C3, 1]'*[C3, 1];
R_aug = 5;

F_aug = -lqr(A_aug, B_aug, Q_aug, R_aug);

F = -lqr(A, B, Q_lqr, R_lqr);
Fi = -lqr(Aaug, Baug, Qaug, Raug);

sys_cl_int = ss(A_aug+B_aug*Fi, B_aug, C_aug, 0);

%% IV.b Analysis of the new Design

disp('Eigenvalues of the closed loop: ');
damp(sys_cl_int);

%% TODO: Complete the Simulink Model

open('cstd2_sim_lqg_int');

%% IV.c Simulation of the new design

sys = ss1;
sim('cstd2_sim_lqg_int');
data_s1 = data; 

figure(3);
t = data(:,1);
plot(t, data(:,2), t, data(:,3), t, data(:,4), t, data(:,5));
legend({'r_1','r_2','y_1','y_2'});
grid('on');

% Simulation with an other plant
sys = ss1;
sim('cstd2_sim_lqg_int');
data_s2 = data; 

figure(4);
t = data(:,1);
plot(t, data(:,2), t, data(:,3), t, data(:,4), t, data(:,5));
legend({'r_1','r_2','y_1','y_2'});
grid('on');
