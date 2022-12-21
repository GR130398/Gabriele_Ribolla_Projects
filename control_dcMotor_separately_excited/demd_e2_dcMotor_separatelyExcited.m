clc 
close all
clear all

%% system data

v_n=600;
omega_n= 314; 
eta =0.9;
tau_a=10e-3;
tau_e=1;
Ve=120;
Ie=1;

M=200*80+10000;
v_max= 60*1000/3600; %max speed
a= v_max/25;  %acceleration

p_acc = M*a*v_max; 
p_tot = p_acc+ p_acc/3;

I_n= p_tot/(eta*v_n);
T_n= p_tot/omega_n;

K= T_n/(I_n*Ie);

Ra = (1-eta)*v_n/I_n;
La= Ra*tau_a;
En= Ve/Ie;

J=M*v_max^2/omega_n^2;
B= (p_acc/3)/omega_n^2;

Re= Ve/Ie;
Le= Re*tau_e;


%% transfer functions

s =tf('s');
%armature

Gia=1/(Ra+s*La);
tauGia=La/Ra;
TaGia=5*tauGia;

%mechanical 
Gw = 1/(B+s*J);
tauGw= J/B;
TauGw=5*tauGw;

%excitation 
Gie = 1/(Re+s*Le);
tauGie=Le/Re;
TauGie=5*tauGie;

T_tram = 25; %time to reach the nominal speed
tauT = T_tram/5;
tspeed= tauT /10;
ws = 1/tspeed; %2 esternal loop
wie= ws/10; %1 external loop
wia=wie/10; % inner loop

tauwia = tauGia/10; 
tauwie= tauwia/10;
tauws= tauwie/10;

%currrent controller 
kp_curr=wia*La;
ki_curr = wia*Ra;

Ria=kp_curr+ki_curr/s;

%excitation controller
kp_exc=wie*Le;
ki_exc=wie*Re;

Rie= kp_exc + ki_exc/s;


%speed controller
kp_speed= ws*J;
ki_speed= ws*B;

Rspeed= kp_speed + ki_speed/s;


C1 = 16.7/314; %rad/s to m/s;
C2 = 314/16.7; % m/s to rad/s
C3 = 314/60; %km/h to rad/s




