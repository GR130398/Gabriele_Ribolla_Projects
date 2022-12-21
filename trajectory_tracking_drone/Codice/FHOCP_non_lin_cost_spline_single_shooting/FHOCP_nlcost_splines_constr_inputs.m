%% comments
clear all
%close all
% clc
%% Model parameters
Jx=0.0058319;
Jy=0.0058319;
Jz=0.0111886;

mass = 0.9272;%(kg)
g=9.81;
B = (mass*g)/4;



% DRONE parameters
c_t = 0.1;
l = 0.13;
c_q = 0.127;
th = [mass;Jx;Jy;Jz;g;c_t;l;c_q];

% weigths
th_cons(1)=1 ;
th_cons(2)=1;
th_cons(3)=1;
th_cons(4)=1;
th_cons(5)=1;
th_cons(6)=500;


% punti circuito
tot_punti = 100;

%% FHOCP parameters - multiple shooting
Ts      =       0.2;               % seconds, input sampling period
Tend    =       3;                 % seconds, terminal time
Np      =       Tend/Ts;           % prediction horizon

% prediction horizon
Np = 11; % punti in avanti



%% optimization problem initialization 
tuneX0 = 0;
x0_u      =     [tuneX0*ones(Np,1);       % speed f1
                tuneX0*ones(Np,1);         % speed f2
                tuneX0*ones(Np,1);         % speed f3
                tuneX0*ones(Np,1);         % speed_f4
                tuneX0*ones(Np,1)];         % speed theta
x0_s = [ ...
%                 tuneX0*ones(Np,1);....       % p
%                 tuneX0*ones(Np,1);....       % q
%                 tuneX0*ones(Np,1);....       % r
%                 tuneX0*ones(Np,1);....       % f1
%                 tuneX0*ones(Np,1);.....      % f2
%                 tuneX0*ones(Np,1);....       % f3
%                 tuneX0*ones(Np,1);....       % f4
%                 tuneX0*ones(Np,1)....       % v_theta
               
];

 x0 = [x0_u;x0_s];                               




% capire quanti punti fare
[xRef2s,yRef2s,C_spline,gradbp]= reference_points_normalization_v1(tot_punti); %(x,Ts,Np,th,);








% Run solver


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
          f_t_in;
          progress_in;
          progress_speed_in];

% initializes inputs
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

%da riattivare per il FHOCP       
y_des = yRef2s(1:Np,1);
x_des = xRef2s(1:Np,1);


y_way = yRef2s(1:Np,1);
x_way = xRef2s(1:Np,1);
desired_progress = gradbp(1,1:Np)';

% x0 = x0_u;
progress_sim = zeros(Np,1);
progress_vec = gradbp;

% vector initialization
nz= 18;
nu = 5;
Nsim = 100;
Zsim_MPC = zeros(nz,Nsim);
u_sim_MPC =zeros(nu,Nsim);
Zsim_MPC(:, 1) = z0;
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

x_spl_c = zeros(Np,4);
y_spl_c = zeros(Np,4);

progress_sim_in = zeros(Np,1);


pos_progress = 17;

indexRefVett = zeros(1,Nsim);
desired_progress_vec = zeros(Np,Nsim);
desired_vec = zeros(5,Nsim);
desired_vec(1,:) = progress_vec(1:Nsim);




time_FFD    =   [0:0.01:(Np-1)*Ts];
Nblock      =   Ts/0.01;
Nsim_FFD    =   length(time_FFD);

z_sim_MPC      =   zeros(18,Nsim_FFD);
z_sim_MPC(:,1) =   z0; 
nu = 5;

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

p_min= -10; 
p_max= 10 ;
q_min = -10;  
q_max = 10;
r_min = -10;
r_max = 10;

speed_f_1_min = - (2*B)/Ts;
speed_f_2_min = -(2*B)/Ts;
speed_f_3_min = -(2*B)/Ts;
speed_f_4_min = -(2*B)/Ts;
speed_f_1_max = (2*B)/Ts;
speed_f_2_max = (2*B)/Ts;
speed_f_3_max = (2*B)/Ts;
speed_f_4_max = (2*B)/Ts;

delta_error = 0;%
delta_error_perc = delta_error/100;
dist_tot_cir =1.860370276320167e+03;
dist_step = dist_tot_cir/(tot_punti);
v_theta_min = 0;
v_theta_max = (1-delta_error_perc)*(dist_step)/Ts;


 speed_p_min =-20;% -dist_step/Ts^2; 
 speed_p_max = 20;%dist_step/Ts^2; 
v_theta_max = speed_p_max*Ts;


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

%     ones(size_1_input,1)*p_min;....
%     ones(size_1_input,1)*q_min;....
%     ones(size_1_input,1)*r_min;....
%     ones(size_1_input,1)*f1_min;....
%     ones(size_1_input,1)*f2_min;....
%     ones(size_1_input,1)*f3_min;....
%     ones(size_1_input,1)*f4_min;....
%     ones(size_1_input,1)*v_theta_min;....


    -ones(size_1_input,1)*speed_f_1_max;...
    -ones(size_1_input,1)*speed_f_2_max;...
    -ones(size_1_input,1)*speed_f_3_max;...
    -ones(size_1_input,1)*speed_f_4_max;...
    -ones(size_1_input,1)*speed_p_max...

%     -ones(size_1_input,1)*p_max;....
%     -ones(size_1_input,1)*q_max;....
%     -ones(size_1_input,1)*r_max;....
%     -ones(size_1_input,1)*f1_max;....
%     -ones(size_1_input,1)*f2_max;....
%     -ones(size_1_input,1)*f3_max;....
%     -ones(size_1_input,1)*f4_max;....
%     -ones(size_1_input,1)*v_theta_max...
    ];


%% per splines function 

%% per splines function 
x_spline = spline(gradbp,xRef2s);
x_spline_coeff = x_spline.coefs;

y_spline = spline(gradbp,yRef2s);
y_spline_coeff = y_spline.coefs;


covariance_xy =cov(xRef2s,yRef2s);
cov_x = cov(x_way);
cov_y = cov(y_way);
matrix_cov = diag([cov_x,cov_y]);
den_din= sqrt(((2*pi)^3)*abs(matrix_cov));
den_din = sqrt(((2*pi)^3)*norm(matrix_cov));
den_din_x = sqrt(((2*pi)^3)*abs(cov_x));
den_din_y = sqrt(((2*pi)^3)*abs(cov_y));
tot_way = size(x_way,1);
tx = zeros(Np,1);
ty = zeros(Np,1);

weight1_din = zeros(Np,2); % ad ogni iterazione lo ri-aggiorno in base al 
%progress sim 
progress_sim = desired_progress;
for i = 1 : Np % da realizzare come da report

%     if (z_sim_MPC(1,ind-1)>= x && )

f_x(i,1) = spline(gradbp,xRef2s,progress_sim(i,1));
f_y(i,1) = spline(gradbp,yRef2s,progress_sim(i,1));


y_des = f_y;
x_des = f_x;


index_ref(i) = find(progress_vec > progress_sim(i,1),1,'first'); 
x_spl_c(i,:) = x_spline_coeff(index_ref(i)-1,:);
y_spl_c(i,:) = y_spline_coeff(index_ref(i)-1,:);
progress_sim_in(i)= progress_vec(index_ref(i)-1);

[f_x_fun,df_x,ddf_x]= spline_f_fd_fdd_v2(progress_sim(i,1),x_spl_c(i,:),progress_sim_in(i,1));
[f_y_fun,df_y,ddf_y]= spline_f_fd_fdd_v2(progress_sim(i,1),y_spl_c(i,:),progress_sim_in(i,1));

tx(i,1) =df_x;
ty(i,1) = df_y;


                for i_dim = 1 : tot_way 
                % altro modo per farlo
                diff_x2_cov = ((f_x(i,1) - x_way(i_dim,1))^2)*cov_x^-1;
                diff_y2_cov = ((f_y(i,1) - y_way(i_dim,1))^2)*cov_y^-1;
                
                arg_exp_x = -0.5*(diff_x2_cov);
                arg_exp_y = -0.5*(diff_y2_cov);
                
                weight_x = exp(arg_exp_x)/(den_din_x);
                weight_y = exp(arg_exp_x)/(den_din_x);
                
                weight1_din(i,1:2)= weight1_din(i,1:2) + [weight_x,weight_y];
       
                
                
                
                end


end


np_x_desired_vec(:,1) =  y_des;
np_y_desired_vec(:,1) =   x_des;
np_index_ref(:,1) = index_ref';

step_1_xy_desired(1,1) = x_des(1,1);
step_1_xy_desired(2,1) = y_des(1,1);

step_1_index_ref(:,1) = index_ref(1);



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
%myoptions.outputfcn     =   @(x)Vehicle_traj(x,Ts,Np,th);

 q = 0;
 p = 0;


 [xstar,fxstar,niter,exitflag,xsequence] =...
     myfmincon(@(x)f_nl_m_nl_v1...
     (x,th,th_cons,weight1_din,y_des,x_des,z0,tx,ty,Ts,Np)...
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
figure,plot(y_sim,x_sim,'b.-',y_des,x_des,'r.-'),grid on, hold on,xlabel('y'),ylabel('x')
legend('sim','des')
hold off


