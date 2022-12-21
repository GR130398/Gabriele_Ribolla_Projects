function q_h = f_ln_st_in_FHOCP_(x,Ts,Np,th,th_cons,y_des,x_des,z0,desired_progress)
%% Build vector of inputs

t_in        =   (0:Ts:(Np-1)*Ts)';

weight1 =th_cons(1) ;
weight2=th_cons(2)  ;
weight3=th_cons(3) ;
weight4= th_cons(4);
weight5=th_cons(5);
weight6=th_cons(6);


u_in(1,:)=x(1:Np,1)';
u_in(2,:)= x(Np+1:2*Np,1)';
u_in(3,:)= x(2*Np+1:3*Np,1)';
u_in(4,:)= x(3*Np+1:4*Np,1)';
u_in(5,:)= x(4*Np+1:5*Np,1)';


assignin('base','z0',z0);
assignin('base','t_in',t_in);
assignin('base','u_in',u_in);
%% Run simulation with FFD
time_FFD    =   [0:0.01:(Np-1)*Ts];
Nblock      =   Ts/0.01;
Nsim_FFD    =   length(time_FFD);

z_sim      =   zeros(18,Nsim_FFD);
z_sim(:,1) =   z0;
for ind=2:Nsim_FFD
    u                  =   u_in(:,1+floor(time_FFD(ind)/Ts));
    zdot               =   linquadcopter_augmented_v1(0,z_sim(:,ind-1),u,th);
    z_sim(:,ind)       =   z_sim(:,ind-1)+Ts/Nblock*zdot;



    z_sim(:,ind)       =   z_sim(:,ind-1)+Ts/Nblock*zdot;
p_sim       =   x(5*Np+1:6*Np,1)';
q_sim       =   x(6*Np+1:7*Np,1)';
r_sim       =   x(7*Np+1:8*Np,1)';


% f1
f1_sim= x(8*Np+1:9*Np,1)';
%f2
f2_sim = x(9*Np+1:10*Np,1)';
%f3
f3_sim = x(10*Np+1:11*Np,1)';
%f4
f4_sim = x(11*Np+1:12*Np,1)';
%v_theta
v_theta = x(12*Np+1:13*Np,1)';


    % p
    z_sim(10,ind-1) = p_sim(1,1+floor(time_FFD(ind)/Ts));
    % q
    z_sim(11,ind-1) =   q_sim(1,1+floor(time_FFD(ind)/Ts));
    %r
    z_sim(12,ind-1) =   r_sim(1,1+floor(time_FFD(ind)/Ts));
    % f1
    z_sim(13,ind-1) =   f1_sim(1,1+floor(time_FFD(ind)/Ts));
    % f2
    z_sim(14,ind-1) =   f2_sim(1,1+floor(time_FFD(ind)/Ts));
    % f3
    z_sim(15,ind-1) =   f3_sim(1,1+floor(time_FFD(ind)/Ts));
    % f4
    z_sim(16,ind-1) =   f4_sim(1,1+floor(time_FFD(ind)/Ts));
    % v_theta
    z_sim(18,ind-1) =  v_theta(1,1+floor(time_FFD(ind)/Ts));


end



;
x_sim       =   z_sim(1,1:Nblock:end)';
y_sim       =   z_sim(2,1:Nblock:end)';
%x(Np*Np+1,1) =  z_sim(10,1:Nblock:end)';
%x(Np*Np+2,1) = z_sim(11,1:Nblock:end)';
%x(Np*Np+3,1) = z_sim(12,1:Nblock:end)';
%x(Np*Np+4,1) = z_sim(15,1:Nblock:end)';
%x(Np*Np+5,1) = z_sim(16,1:Nblock:end)';
%x(Np*Np+6,1) = z_sim(17,1:Nblock:end)';
%x(Np*Np+7,1)= z_sim(18,1:Nblock:end)';
%Z_sim       =   z_sim(3,1:Nblock:end)';

%x_dot_sim       =   z_sim(4,1:Nblock:end)';
%y_dot_sim       =   z_sim(5,1:Nblock:end)';
%z_dot_sim       =   z_sim(6,1:Nblock:end)';

%psi_sim       =   z_sim(7,1:Nblock:end)';
%theta_sim       =   z_sim(8,1:Nblock:end)';
%phi_sim       =   z_sim(9,1:Nblock:end)';

p_sim       =   z_sim(10,1:Nblock:end)';
q_sim       =   z_sim(11,1:Nblock:end)';
r_sim       =   z_sim(12,1:Nblock:end)';

%augmented states
progress_sim        = z_sim(17,1:Nblock:end)';
% to define the position of the quadcopter w.r.t the trajectory
progress_speed_sim= z_sim(18,1:Nblock:end)';
%f1_sim = z_sim(15,1:Nblock:end)';
%f2_sim = z_sim(16,1:Nblock:end)';
%f3_sim = z_sim(17,1:Nblock:end)';
%f4_sim = z_sim(18,1:Nblock:end)';


% costo considerato dall'errore totale tra x e y dei punti
e_c_y = (y_des-y_sim)'*(y_des-y_sim); % errore lungo l'asse x
e_c_x = (x_des - x_sim)'*(x_des- x_sim); % errore lungo l'asse y
normErroreTot = sqrt(e_c_y+e_c_x);
cost1 = normErroreTot^2;
cost2 = (desired_progress - progress_sim)'*(desired_progress - progress_sim);


%delta angular speeds cost 
cost3 = p_sim'*p_sim + q_sim'*q_sim + r_sim'*r_sim ; % data da tutti e tre

%speed progress  cost
cost4 = u_in(5,:)*u_in(5,:)';
%speed f1,f2,f3,f4 cost
cost5 = u_in(1,:)*u_in(1,:)'+u_in(2,:)*u_in(2,:)'+u_in(3,:)*u_in(3,:)'+u_in(4,:)*u_in(4,:)';

%speed progress cost
mu = 500;
cost6 = - mu *sum(progress_speed_sim,'all'); 



cost_tot = weight1*cost1 +weight2*cost2+ weight3*cost3 +weight4* cost4 ...
    +weight5* cost5 +weight6* cost6 ;
q_h          =   cost_tot;
end





%% cosa inutile della funzione che intasa il tutto

%vettore tangente
%pdThetaK = 
%fun = pdThetaK;
%xk;
%method;
%dx;
%[fxk,gradient] = mygradient(fun,xk,method,dx);
%vettTang = gradient;
%eLong = 
%cost2  = e_c_x + e_c_y; 

%costo 2 sull errore tra progress desired e progress sim
%cost2 = (desired_progress - progress_sim)'*(desired_progress - progress_sim);

%% for cycle for the  prediction

%% compute the el error (projection of e(theta_k) w.r.t. the tangent) cost1
%cost1  = 
%% compute the contour error cost2 

%{
i_higher_value=zeros(length(progress_sim),1);
x_des = zeros(length(progress_sim),1);
y_des = zeros(length(progress_sim),1);
for k  = 1 : length(progress_sim)
b = progress_sim(k);
%i_higher = find(data >= x,1);

i_higher = find_vector_index(data,b);
i_higher_values(k)=i_higher;
%i_higher = i(1,2);
%ind = i_higher;
%a = C(ind,1);
%b = C(ind,2);
%c = C(ind,3);
%d = C(ind,4);
%x1 = data(ind-1);
%x2 = data(ind);
%x = x1 + 0.5*(x2-x1);
%[f_spline,df_spline,ddf_spline]=spline_f_fd_fdd(x,a,b,c,d,x1,x2);

xRef2s(i_higher)
%i_higher
x_des(k) = xRef2s(i_higher,1);
y_des(k) = yRef2s(i_higher,1);
%}

%end 
%}
%{
dataTrack = load('LVMS_ORC_NV.mat'); %Las Vegas Motor Speedway - Outside Road Course - North Variant 

%refPose = data.ActorSpecifications(1,46).Waypoints;
% define data for velocity lookup table
%lookUpt = readmatrix('velocityDistributionHighway.xlsx');
%xlt = lookUpt(2:42,1);
%ylt = lookUpt(1,2:31);
%vel = lookUpt(2:42,2:31)*4/5;
% specify simulation stop time
Tend = 45*5/4;

%% define reference points
%xRef = refPose(:,1);
%yRef = -refPose(:,2);
xRef = dataTrack.Inside(:,[1]);
yRef=  dataTrack.Inside(:,[2]);
refPose(:,1) = xRef;
refPose(:,2) = yRef;

%{ 
outside points
x_out= data_track.Outside(:,[1]);
y_out= data_track.Outside(:,[2]);
%}
%% define vehicle parameters used in the models

X_o = xRef(1); % initial vehicle position
Y_o = yRef(1); % initial vehicle position 
psi_o = 88.5*(pi/180); % it's an important step to initialize yaw angle

%% calculating reference pose vectors
% Based on how far the vehicle travels, the pose is generated using 1-D
% lookup tables.

%distance = sqrt(xRef'*xRef-yRef'*yRef);
% calculate distance vector
distancematrix = squareform(pdist(refPose));
distancesteps = zeros(length(refPose)-1,1);
for i = 2:length(refPose)
    distancesteps(i-1,1) = distancematrix(i,i-1);
end
totalDistance = sum(distancesteps); % Total traveled distance
distbp = cumsum([0; distancesteps]); % Distance for each waypoint
gradbp = linspace(0,totalDistance,(Nsim_FFD-1)/Nblock +1); % Linearize distance

% linearize X and Y vectors based on distance
xRef2 = interp1(distbp,xRef,gradbp,'pchip');
yRef2 = interp1(distbp,yRef,gradbp,'pchip');
yRef2s = smooth(gradbp,yRef2);
xRef2s = smooth(gradbp,xRef2);
%}
%end
%}
%i_higher_value;


