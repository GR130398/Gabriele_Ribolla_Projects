function [zdot]=linquadcopter_augmented_v1(t,z,u,th)

%{

cambio il settaggio degli input con speep_progress_ --> 5

%}


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Read parameters, states and inputs
%{
% u= agumented inputs
  d = tau,roll,yaw torques
%}
% Parameters
mass = th(1,1);
Jx = th(2,1);
Jy = th(3,1);
Jz = th(4,1);
g = th(5,1);
c_t = th(6,1);
l = th(7,1);
c_q = th(8,1);
% da aggiungere i parametri mancanti
% States
X        =       z(1,1);    % inertial X position (m)
Y        =       z(2,1);    % inertial Y position (m)
Z        =       z(3,1);    % inertial Z position (m)

xbdot     =       z(4,1);    % body x velocity (m/s)
ybdot     =       z(5,1);    % body y velocity (m/s)
zbdot     =       z(6,1);    % body z velocity (m/s)

phi      =       z(7,1);    % yaw angle   (rad)
theta    =       z(8,1);    % pitch angle (rad)
psi      =       z(9,1);    % roll angle  (rad)

speed_phi        =       z(10,1);   % inertial p rate (rad/s)
speed_the        =       z(11,1);   % inertial q rate (rad/s)
speed_psi        =       z(12,1);   % inertial r rate (rad/s)

% additional states
f1 = z(13,1);
f2 = z(14,1);
f3 = z(15,1);
f4 = z(16,1);
progress =       z(17,1); % to define the position of the quadcopter w.r.t the trajectory
progress_speed = z(18,1); 
% Inputs
T         =       (f1+f2+f3+f4);     % thrust force (N)
tauroll   =       l*(f2-f4);%d(1,1);     % roll torque  (N*m)
taupitch  =       l*(-f1+f3);     % pitch torque (N*m)
tauyaw    =       (c_q/c_t)*(-f1+f2-f3+f4); %d(3,1);     % yaw torque   (N*m)

%augmented inputs 

speed_f1= u(1,1); 
speed_f2= u(2,1);
speed_f3= u(3,1);
speed_f4= u(4,1);
progress_speed_diff = u(5,1); 
% %semplifications
% cp=cos(psi);
% sp=sin(psi);
% ct=cos(theta);
% st=sin(theta);
% cf=cos(phi);
% sf=sin(phi);

%% Model equations
zdot(1,1)  =   xbdot;
zdot(2,1)  =   ybdot;
zdot(3,1)  =   zbdot;
zdot(4,1)  =   g*theta;
zdot(5,1)  =   -g*phi;
zdot(6,1)  =   T/mass;
zdot(7,1)  =   speed_phi;
zdot(8,1)  =   speed_the;
zdot(9,1)  =   speed_psi;
zdot(10,1) =   tauroll/Jx;
zdot(11,1) =   taupitch/Jy;
zdot(12,1) =   tauyaw/Jz;

%thrust_speed
zdot(13,1) = speed_f1; %f1 
zdot(14,1) = speed_f2; %f2
zdot(15,1)= speed_f3; %f3
zdot(16,1)= speed_f4; %f4

%augmented state dynamics
zdot(17,1) = progress_speed; %progress_speed
zdot(18,1) = progress_speed_diff; %progress_acceleration

