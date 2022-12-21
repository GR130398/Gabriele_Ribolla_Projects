%% MPC quad 
% ln model of us (usando linquadcopter_aug v1)
% nl cost function no splines
% Constraints on inputs
% CHANGED new tipe of given a reference 
% constraints of f without considering tau yaw etc....
% speed_p max(20) e speed_p min(-20) e v_theta_min(0) e v_theta_max(20*Ts) cambiati


clear all
close all
clc

%% Model parameters and Initialize optimization variables (x0)
Jx=0.0058319;
Jy=0.0058319;
Jz=0.0111886;
I_drone = [ ...
            Jx, 0, 0;
            0, Jy, 0;
            0, 0, Jz;
          ] ; % (kg.m^2)

mass = 0.9272;%(kg)
g=9.81;
B = (mass*g)/4;
c_t = 0.1;
l = 0.13;
c_q = 0.127;
%% per simulazione
Tend = 5.4; % tempo di fine simulazione
Ts = 0.3; % tempo di campinamento
Np = floor(Tend/Ts); % 18            % prediction horizon
Nsim = 100;
nu = 5;
nz = 18;
ny = 2;

th = [mass;Jx;Jy;Jz;g;c_t;l;c_q];

%% per referrence
tot_punti = 100;

%% per la cost function
weight1 = 1e3;
weight2 = 0;
weight3 = 1;
weight4 = 1;
weight5 = 1;
weight6 = 1;
weight7 = 0; % costo sulla non troppo differenza tra uno step e l'altro per
weight8 = 1;             % per il progress sim

%% constraints values
% max e min x,y presi dal vettore di ref
x_min = -309.1718;
x_max = -2.7387;
y_min =  -528.7749;
y_max = 98.0883;


Z_ref= 10;

% constraints su progress sim, viene aggiornato nel main, in base a
% progress_sim ed utilizzato poi nella fun_constraint_v4 per la matrice dei
% constraints (non usata in questa versione, ma in altre versioni precendenti)
theta_min = 0;
theta_max =0;



f1_min = 0; 
f1_max = 2*B;
f2_min = 0; 
f2_max = 2*B;
f3_min = 0; 
f3_max = 2*B;
f4_min = 0; 
f4_max = 2*B;


speed_f_1_min = - (2*B)/Ts;
speed_f_2_min = -(2*B)/Ts;
speed_f_3_min = -(2*B)/Ts;
speed_f_4_min = -(2*B)/Ts;
speed_f_1_max = (2*B)/Ts;
speed_f_2_max = (2*B)/Ts;
speed_f_3_max = (2*B)/Ts;
speed_f_4_max = (2*B)/Ts;

p_min= -10; 
p_max= 10 ;
q_min = -10;  
q_max = 10;
r_min = -10;
r_max = 10;

delta_error = 0;%
delta_error_perc = delta_error/100;
dist_tot_cir =1.860370276320167e+03;
dist_step = dist_tot_cir/(tot_punti*100);
v_theta_min = 0;


% v_theta_max = (1-delta_error_perc)*(dist_step)/Ts;
% speed_p_min = -(1+delta_error_perc)*(dist_step)/Ts^2;
% speed_p_max = (1-delta_error_perc)*(dist_step)/Ts^2 ;

% abbiamo trovato meglio usare questi constraints
speed_p_min = -20;
speed_p_max = 20; 
v_theta_max = speed_p_max*Ts;

z_min = 0;
z_max = 1e8;

%% RUN MPC
% Algorithm 7.2.1 - Prototype of NMPC algorithm
% 1.	Get z(t) (e.g. from direct measurements or from an observer, for example 
%     realized with MHE, see Section 7.3.2);
% 2.	Solve the FHOCP (7.3) with z0 = z(t), let U∗ be the found minimizer;
% 3.	Apply to the plant the input u(t) = u∗(0|t);
% 4.	Set t = t + 1, go to 1.

%% A define the 3D path V take the data from 
dataTrack = load('LVMS_ORC_NV.mat'); %Las Vegas Motor Speedway - Outside Road Course - North Variant 

% define reference points

xRef = dataTrack.Inside(:,[1]);
yRef=  dataTrack.Inside(:,[2]);
refPose(:,1) = xRef;
refPose(:,2) = yRef;

distancematrix = squareform(pdist(refPose));
distancesteps = zeros(length(refPose)-1,1);
for i = 2:length(refPose)
    distancesteps(i-1,1) = distancematrix(i,i-1);
end
totalDistance = sum(distancesteps); % Total traveled distance
distbp = cumsum([0; distancesteps]); % Distance for each waypoint
gradbp = linspace(0,totalDistance,tot_punti); % Linearize distance

% linearize X and Y vectors based on distance
xRef2 = interp1(distbp,xRef,gradbp,'pchip');
yRef2 = interp1(distbp,yRef,gradbp,'pchip');
yRef2s = smooth(gradbp,yRef2);
xRef2s = smooth(gradbp,xRef2);

% to set as first point of the circuit (0,0)
xRef2s = xRef2s -xRef2s(1);
yRef2s = yRef2s -yRef2s(1);

%% B compute splines V from the reference normalization and Spline coeffients
PP = spline(xRef2s,yRef2s);
C_splines = PP.coefs;
%f(x) = (a(x-x 1)^3)+(b(x-x1)^2)+(c(x-x1))+(d);
%% C define a vector with arc lenght (linearized)
% (between 2 desired points) and with

for ind = 2 : tot_punti
x1 = xRef2s(ind-1,1);
x2 = xRef2s(ind,1);
y1 = yRef2s(ind-1,1);
y2 = yRef2s(ind,1);
    arc_lenght(ind) =  sqrt((x1-x2)^2+(y1-y2)^2);
end

%% system initialization
% states
x_0 = xRef2s(1); % mettendolo come al caso del FHOCP non funziona
y_0  = yRef2s(1);
z_0 = 0;

phi_0= 0;
the_0 =0;
psi_0 = 0;

v_x_0 = 0;
v_y_0 = 0;
v_z_0 = 0;

w_x_0 = 0;
w_y_0 = 0;
w_z_0 = 0;

f1_0 = 0;
f2_0=  0;
f3_0=  0;
f4_0=  0;

theta_0 = 0;
v_theta_0 =0;

z_0 =[x_0;y_0;z_0;phi_0;the_0;psi_0;v_x_0;v_y_0;v_z_0;w_x_0;w_y_0;...
    w_z_0;f1_0;f2_0;f3_0;f4_0;theta_0;v_theta_0];

% inputs
delta_v_theta = 0;
delta_f_1 = 0;
delta_f_2 = 0;
delta_f_3 = 0;
delta_f_4 = 0;

u0(1,1) = delta_f_1 ;
u0(2,1) = delta_f_2 ;
u0(3,1) = delta_f_3 ;
u0(4,1) = delta_f_4 ;
u0(5,1) = delta_v_theta ;

%% optimization problem initialization 
tuneX0 = 0;
x0_u      =     [tuneX0*ones(Np,1);       % speed f1
                tuneX0*ones(Np,1);         % speed f2
                tuneX0*ones(Np,1);         % speed f3
                tuneX0*ones(Np,1);         % speed_f4
                tuneX0*ones(Np,1)];         % speed theta
x0_s = [ ...
    %% per multimple shooting
%                 tuneX0*ones(Np,1);....       % p
%                 tuneX0*ones(Np,1);....       % q
%                 tuneX0*ones(Np,1);....       % r
%                 tuneX0*ones(Np,1);....       % f1
%                 tuneX0*ones(Np,1);.....       % f2
%                 tuneX0*ones(Np,1);....       % f3
%                 tuneX0*ones(Np,1);....       % f4
%                 tuneX0*ones(Np,1);....       % v_theta
                % theta
                 %tuneX0*ones(Np,1);....
];

 x0 = [x0_u;x0_s];
% x0 = x0_u;
progress_sim = zeros(Np,1);
progress_vec = gradbp;

% vector initialization
Zsim_MPC = zeros(nz,Nsim);
u_sim_MPC =zeros(nu,Nsim);
Zsim_MPC(:, 1) = z_0;
u_sim_MPC(:,1) = u0;

% per ref
index_ref=zeros(1,Np);
x2 = zeros(1,Np);
y2 = zeros(1,Np);
progress_pred = zeros(Np,1);

np_x_desired_vec= zeros(Np,Nsim) ;
np_y_desired_vec =zeros(Np,Nsim) ;
np_index_ref= zeros(Np,Nsim);

step_1_xy_desired = zeros(2,Nsim);
step_1_xy_sim = zeros(2,Nsim);
step_1_index_ref = zeros(1,Nsim);

f_y = zeros(Np,1);
f_x = zeros(Np,1);
%per FFD
time_FFD    =   [0:0.01:(Np-1)*Ts];
Nblock      =   Ts/0.01;
Nsim_FFD    =   length(time_FFD);

z_pred      =   zeros(nz,Nsim_FFD);
z_pred(:,1) =   z_0;

pos_progress = 17;

indexRefVett = zeros(1,Nsim);
desired_progress_vec = zeros(Np,Nsim);
desired_vec = zeros(5,Nsim);
desired_vec(1,:) = progress_vec(1:Nsim);


%% Solution -  BFGS
% Initialize solver options
myoptions               =   myoptimset;
myoptions.Hessmethod  	=	'BFGS';
myoptions.gradmethod  	=	'CD';
myoptions.graddx        =	2^-17;
myoptions.tolgrad    	=	1e-8;
myoptions.ls_beta       =	0.5;
myoptions.ls_c          =	.1;
myoptions.ls_nitermax   =	1e2;
myoptions.nitermax      =	1e3;
myoptions.xsequence     =	'on';
myoptions.outputfcn     =   @(x)quad_lin_traj_v1(x,xRef2s,yRef2s,th,z_0,Ts,Np);



%% Inequality matrix for optimization constraints (fun_constraint_v4)
size_x0 = size(x0,1);
size_eye = size_x0;
size_1_input = Np;
C_cons= [eye(size_eye);-eye(size_eye)];

d_cons= [...
    ones(size_1_input,1)*speed_f_1_min;....
    ones(size_1_input,1)*speed_f_2_min;...
    ones(size_1_input,1)*speed_f_3_min;...
    ones(size_1_input,1)*speed_f_4_min;....
    ones(size_1_input,1)*speed_p_min;....
%% per multiple shooting
%     ones(size_1_input,1)*r_min;....
%     ones(size_1_input,1)*q_min;....
%     ones(size_1_input,1)*p_min;....
%     ones(size_1_input,1)*f1_min;....
%     ones(size_1_input,1)*f2_min;....
%     ones(size_1_input,1)*f3_min;....
%     ones(size_1_input,1)*f4_min;....
%     ones(size_1_input,1)*v_theta_min;....

%     ones(size_1_input,1)*theta_min;....


    -ones(size_1_input,1)*speed_f_1_max;...
    -ones(size_1_input,1)*speed_f_2_max;...
    -ones(size_1_input,1)*speed_f_3_max;...
    -ones(size_1_input,1)*speed_f_4_max;...
    -ones(size_1_input,1)*speed_p_max...
% 
%     -ones(size_1_input,1)*r_max;....
%     -ones(size_1_input,1)*q_max;....
%     -ones(size_1_input,1)*p_max;....
%     -ones(size_1_input,1)*f1_max;....
%     -ones(size_1_input,1)*f2_max;....
%     -ones(size_1_input,1)*f3_max;....
%     -ones(size_1_input,1)*f4_max;....
%     -ones(size_1_input,1)*v_theta_max;....

%     -ones(size_1_input,1)*theta_max;....
    ];

q  = 0;

%% MPCC loop
for ind = 2 : Nsim
    fprintf('sim : %d \n',ind-1);
    % 1.	Get z(t) (e.g. from direct measurements or from an observer, for example 
%     realized with MHE, see Section 7.3.2);


for i = 1 : Np 

f_x(i,1) = spline(gradbp,xRef2s,progress_sim(i,1));
f_y(i,1) = spline(gradbp,yRef2s,progress_sim(i,1));


y_des = f_y;
x_des = f_x;


end




np_x_desired_vec(:,ind-1) =  y_des;
np_y_desired_vec(:,ind-1) =   x_des;
np_index_ref(:,ind-1) = index_ref';

step_1_xy_desired(1,ind-1) = x_des(1,1);
step_1_xy_desired(2,ind-1) = y_des(1,1);
step_1_xy_sim(1:2,ind-1) = Zsim_MPC(1:2,ind-1);
step_1_index_ref(:,ind-1) = index_ref(1);
desired_vec(:,ind-1) = [x_des(1,1);y_des(1,1);Zsim_MPC(1:2,ind-1);index_ref(1)];



% 2.	Solve the FHOCP (7.3) with z0 = z(t), let U∗ be the found minimizer;
weight2 = 0; % remove the cost on the progress desired and sim 



 th_wgt = [weight1,weight2,weight3,weight4,weight5,weight6,weight7,weight8];
[xstar,fxstar,niter,exitflag,xsequence] =...
myfmincon(@(x)...
quad_nl_cost_constr_v2(x,th,th_wgt,y_des,x_des,Zsim_MPC(:,ind-1),Ts,Np)...
,x0,[],[],C_cons,d_cons,0,q,myoptions);




% 3.	Apply to the plant the input u(t) = u∗(0|t);
x0 = xstar; % da valutare se lasciare o meno


u(1,:)=xstar(1,1)';
u(2,:)= xstar(Np+1,1)';
u(3,:)= xstar(2*Np+1,1)';
u(4,:)= xstar(3*Np+1,1)';
u(5,:)= xstar(4*Np+1,1)';
u_sim_MPC(:,ind-1)= u; 



[zdot]=linquadcopter_augmented_v1(0,Zsim_MPC(:, ind-1),u_sim_MPC(:,ind-1),th);

Zsim_MPC(:,ind) = Zsim_MPC(:, ind-1) + Ts*zdot;



% %% simulate the system applying both the inputs to use for the referecence
u_in(1,:)=xstar(1:Np,1)';
u_in(2,:)= xstar(Np+1:2*Np,1)';
u_in(3,:)= xstar(2*Np+1:3*Np,1)';
u_in(4,:)= xstar(3*Np+1:4*Np,1)';
u_in(5,:)= xstar(4*Np+1:5*Np,1)';

z_pred(:,1) = Zsim_MPC(:,ind-1);
for ind_in=2:Nsim_FFD
    u_pred                  =   u_in(:,1+floor(time_FFD(ind_in)/Ts));

[zdot_pred]=linquadcopter_augmented_v1(0,z_pred(:,ind_in-1),u_pred,th);
z_pred(:,ind_in) = z_pred(:, ind_in-1) + (Ts/Nblock)*zdot_pred;
end


progress_sim        = z_pred(pos_progress,1:Nblock:end)';

% 4.	Set t = t + 1, go to 1. 
end


%% Plot (X,Y) trajectory and constraints
X_sim       =   Zsim_MPC(1,1:ind-1);
Y_sim       =   Zsim_MPC(2,1:ind-1);




figure,plot(Y_sim,X_sim,'b.-',...
    yRef2s,xRef2s,'r.-')...
    ,grid on, hold on,xlabel('x'),ylabel('y')
legend('sim','circuit')
hold off


