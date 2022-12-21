%% comments
%{
- cambiati i speed_p_min(-20); speed_p_max(20)
%}
clear all
close all
clc
%% Model parameters
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



% DRONE parameters
c_t = 0.1;
l = 0.13;
c_q = 0.127;
th = [mass;Jx;Jy;Jz;g;c_t;l;c_q];

caso_grafici = 0; % durante la simulazione


%% FHOCP parameters - single shooting
Ts      =       0.5;               % seconds, input sampling period
Tend    =       1;                 % seconds, terminal time
Np      =       Tend/Ts;           % prediction horizon

% prediction horizon
Np = 11; % punti in avanti
% fino a 8 molto bene con costo 2


%% Initialize optimization variables

tuneX0 = 1; % potremmo cambiarlo 
x0_u      =       [tuneX0*ones(Np,1);        % speed theta
                tuneX0*ones(Np,1);         % speed f1
                tuneX0*ones(Np,1);         % speed f2
                tuneX0*ones(Np,1);         % speed f3
                tuneX0*ones(Np,1)];        % speed f4
                                       
x0_s = [ ...
                tuneX0*ones(Np,1);....       % p
                tuneX0*ones(Np,1);....       % q
                tuneX0*ones(Np,1);....       % r
                tuneX0*ones(Np,1);....       % f1
                tuneX0*ones(Np,1);.....       % f2
                tuneX0*ones(Np,1);....       % f3
                tuneX0*ones(Np,1);....       % f4
                tuneX0*ones(Np,1);....       % v_theta
    
];

 x0 = [x0_u;x0_s];


% capire quanti punti fare
tot_punti = 100;
[xRef2s,yRef2s,C_spline,gradbp]= reference_points_normalization_v1(tot_punti);
%iHigher=find_vector_index(gradbp,50);
% Run solver

 %i_higher = find_vector_index(gradbp,100)
Xe_init   = [xRef2s(tot_punti,1);yRef2s(tot_punti,1);0];% (m) initial position in inertial axes
Vb_init   = [0;0;0]; % (m/sec) initial velocity in BODY axes
eul_init  = [0;0;0]; % (rad) initial euler angles (yaw,pitch,roll)
wb_init   = [0;0;0]; % (rad/sec) initial body rates
progress_in =0; % to define the position of the quadcopter w.r.t the trajectory
progress_speed_in = 0; 
f1_in = 0;
f2_in = 0;
f3_in = 0;
f4_in= 0;
f_t_in =[f1_in;f2_in;f3_in;f4_in];

z0 = [ Xe_init;
           Vb_init;
           eul_init;
           wb_init;
           progress_in;
           f_t_in;
           progress_speed_in;
           ];



%da riattivare per il FHOCP       
y_des = yRef2s(1:Np,1);
x_des = xRef2s(1:Np,1);
desired_progress = gradbp(1,1:Np)';

time_FFD    =   [0:0.01:(Np-1)*Ts];
Nblock      =   Ts/0.01;
Nsim_FFD    =   length(time_FFD);

z_sim_MPC      =   zeros(18,Nsim_FFD);
z_sim_MPC(:,1) =   z0; 
nu = 5;
u_sim_MPC            =   zeros(nu,length(yRef2s)); % definizione vettore input 

%% Constraints

% per constraints
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
dist_step = dist_tot_cir/(tot_punti);
v_theta_min = 0;
v_theta_max = (1-delta_error_perc)*(dist_step)/Ts;
speed_p_min = -(1+delta_error_perc)*(dist_step)/Ts^2;
speed_p_max = (1-delta_error_perc)*(dist_step)/Ts^2 ;


% Bounds on input variables
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

    ones(size_1_input,1)*p_min;....
    ones(size_1_input,1)*q_min;....
    ones(size_1_input,1)*r_min;....
    ones(size_1_input,1)*f1_min;....
    ones(size_1_input,1)*f2_min;....
    ones(size_1_input,1)*f3_min;....
    ones(size_1_input,1)*f4_min;....
    ones(size_1_input,1)*v_theta_min;....




    -ones(size_1_input,1)*speed_f_1_max;...
    -ones(size_1_input,1)*speed_f_2_max;...
    -ones(size_1_input,1)*speed_f_3_max;...
    -ones(size_1_input,1)*speed_f_4_max;...
    -ones(size_1_input,1)*speed_p_max...
 
    -ones(size_1_input,1)*p_max;....
    -ones(size_1_input,1)*q_max;....
    -ones(size_1_input,1)*r_max;....
    -ones(size_1_input,1)*f1_max;....
    -ones(size_1_input,1)*f2_max;....
    -ones(size_1_input,1)*f3_max;....
    -ones(size_1_input,1)*f4_max;....
    -ones(size_1_input,1)*v_theta_max;....


    ];

q  = 0;
p = 0;

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
switch caso_grafici
    case 1
myoptions.outputfcn     =   @(x)quad_lin_traj_v1(x,xRef2s,yRef2s,th,z0,Ts,Np);
end

weight1 = 1;
weight2= 1 ;
weight3= 1 ;
weight4= 1;
weight5= 1 ;
weight6= 500;

th_cons= [ weight1, weight2, weight3, weight4, weight5, weight6];


 z_prev = z0;
 [xstar,fxstar,niter,exitflag,xsequence] = ...
     myfmincon(@(x)f_ln_st_in_FHOCP_...
     (x,Ts,Np,th,th_cons,y_des,x_des,z_prev,desired_progress)...
     ,x0,[],[],C_cons,d_cons,p,q,myoptions);


x0 = xstar;
u(1,:)=xstar(1:Np,1)';
u(2,:)= xstar(Np+1:2*Np,1)';
u(3,:)= xstar(2*Np+1:3*Np,1)';
u(4,:)= xstar(3*Np+1:4*Np,1)';
u(5,:)= xstar(4*Np+1:5*Np,1)';
u_sim_MPC= u; 


% applicare gli inputs ottimali trovati al sistema e farne la simulazione
time_FFD    =   [0:0.01:(Np-1)*Ts];
Nblock      =   Ts/0.01;
Nsim_FFD    =   length(time_FFD);

nS = 18;


z_sim = zeros(nS,Nsim_FFD);
z_sim(:,1) = z0;
for ind=2:Nsim_FFD
    u                  =   u_sim_MPC(:,1+floor(time_FFD(ind)/Ts));
    zdot               =   linquadcopter_augmented_v1(0,z_sim(:,ind-1),u,th);
    z_sim(:,ind)       =   z_sim(:,ind-1)+Ts/Nblock*zdot;
end

x_sim       =   z_sim(1,1:end)';
y_sim       =   z_sim(2,1:end)';




%% Plot the results
figure(6),plot(y_sim,x_sim,'b.-',y_des,x_des,'r.-'),grid on, hold on,xlabel('y'),ylabel('x')
legend('sim','des')
hold off


