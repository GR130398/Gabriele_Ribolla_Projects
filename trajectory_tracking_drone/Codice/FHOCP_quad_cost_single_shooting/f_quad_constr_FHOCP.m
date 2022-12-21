function cost_tot = f_quad_constr_FHOCP(x,Ts,Np,th,th_cons,y_des,x_des,z0,desired_progress)
%% Build vector of inputs
%{
- constraints : solo inputs 
- mod: linear
- quadratic cost
- weights : constant
%}

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
u_in(5,:)= x(4*Np+1:end,1)';


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
end


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
mu = 1;
cost6 = - mu *sum(progress_speed_sim,'all'); 

cost_tot = ...
    weight1*cost1 +weight2*cost2+ weight3*cost3 ....
    +weight4*cost4 + weight5*cost5 + weight6*cost6 ;

end







