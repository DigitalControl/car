%
% Velocity controller of a small robotic car's wheel
%
% Based on:
%  http://ctms.engin.umich.edu/CTMS/index.php?example=MotorSpeed&section=ControlDigital
%
clear all;
close all;

pkg load control;
s = tf('s');

% Model parameters
J = 0.01;  % kg*m^2
b = 0.1;   % N*m*s
Ke = 0.01; % V/sec/rad
Kt = Ke;   % N*m/A
K = Ke;    % Since Ke=Kt, just use K
R = 1;     % Ohm
L = 0.5;   % H

% State-space representation
A = [-b/J K/J; -K/L -R/L];
B = [0 1/L]';
C = [1 0];
D = [0];

sys = ss(A, B, C, D);

% Discretize
Ts = 0.05;
dsys = c2d(sys, Ts, 'zoh');

% Open-loop analysis
%zpk(dsys)
%bode(dsys);

% Closed-loop when digitized
figure;
sys_cl = feedback(dsys, 1);
[y,t] = step(sys_cl, 12);
stairs(t,y);
xlabel('Time (s)');
ylabel('Velocity (rad/s)');
title('Stairstep Response: Original');

% PID Controller
Kp = 100;
Ki = 200;
Kd = 10;

C = Kp + Ki/s + Kd*s;
dC = c2d(C, Ts, 'tustin');

% Closed-loop analysis
%zpk(dC)
%bode(dC);

figure;
sys_cl = feedback(dC*dsys, 1);
[y,t] = step(sys_cl, 12);
stairs(t,y);
xlabel('Time (s)');
ylabel('Velocity (rad/s)');
title('Stairstep Response: PID controller');

% Analsys, since system goes wildly out of control
figure;
rlocus(dC*dsys);
axis([-1.5 1.5 -1 1]);
title('Root Locus of Compensated System');

% Check to see that system is controllable
sys_rank = rank(ctrb(A,B))

% Check to see that system is observable
ob = obsv(sys);
observability = rank(ob)

