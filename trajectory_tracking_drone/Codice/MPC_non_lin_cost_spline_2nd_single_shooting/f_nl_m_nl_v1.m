function q_h = f_nl_m_nl_v1...
    (x,th,th_cons,weight1_din,y_des,x_des,z0,df_x,df_y,Ts,Np)

%{
% considera il constraint sul theta sim--> tolto
% mod : ln 
% cost  :  nl 
% Constraints: Single shooting
- weight dynamic

%}
%% Build vector of inputs


t_in        =   (0:Ts:(Np-1)*Ts)';
weight1 =th_cons(1) ;
weight2=th_cons(2)  ;
weight3=th_cons(3) ;
weight4= th_cons(4);
weight5=th_cons(5);
weight6=th_cons(6);




u_in(1,:)=x(1:Np,1)'; % speed_f1(1)-->speed_f1(Np)
u_in(2,:)= x(Np+1:2*Np,1)';
u_in(3,:)= x(2*Np+1:3*Np,1)';
u_in(4,:)= x(3*Np+1:4*Np,1)';
u_in(5,:)= x(4*Np+1:5*Np,1)';





assignin('base','z0',z0);
assignin('base','t_in',t_in);
assignin('base','u_in',u_in);
%% Run simulation with ODE

n_z = length(z0);
n_u = length(u_in);

time_FFD    =   [0:0.01:(Np-1)*Ts];
Nblock      =   Ts/0.01;
Nsim_FFD    =   length(time_FFD);

z_sim      =   zeros(n_z,Nsim_FFD);
z_sim(:,1) =   z0;
for ind=2:Nsim_FFD
    u_ffd                  =   u_in(:,1+floor(time_FFD(ind)/Ts));

[zdot_fun]=linquadcopter_augmented_v1(0,z_sim(:,ind-1),u_ffd,th);



% [zdot]=linquadcopter_augmented_v1(0,z_sim(:,ind-1),u,th);
    z_sim(:,ind)       =   z_sim(:,ind-1)+Ts/Nblock*zdot_fun;

    %% for multiple shooting
% p_sim       =   x(5*Np+1:6*Np,1)';
% q_sim       =   x(6*Np+1:7*Np,1)';
% r_sim       =   x(7*Np+1:8*Np,1)';
% 
% 
% % f1
% f1_sim= x(8*Np+1:9*Np,1)';
% %f2
% f2_sim = x(9*Np+1:10*Np,1)';
% %f3
% f3_sim = x(10*Np+1:11*Np,1)';
% %f4
% f4_sim = x(11*Np+1:12*Np,1)';
% %v_theta
% v_theta = x(12*Np+1:13*Np,1)';
% % theta_sim 
% % theta_sim = x(13*Np+1:14*Np,1)';
% 
%     % p
%     z_sim(10,ind-1) = p_sim(1,1+floor(time_FFD(ind)/Ts));
%     % q
%     z_sim(11,ind-1) =   q_sim(1,1+floor(time_FFD(ind)/Ts));
%     %r
%     z_sim(12,ind-1) =   r_sim(1,1+floor(time_FFD(ind)/Ts));
%     % f1
%     z_sim(13,ind-1) =   f1_sim(1,1+floor(time_FFD(ind)/Ts));
%     % f2
%     z_sim(14,ind-1) =   f2_sim(1,1+floor(time_FFD(ind)/Ts));
%     % f3
%     z_sim(15,ind-1) =   f3_sim(1,1+floor(time_FFD(ind)/Ts));
%     % f4
%     z_sim(16,ind-1) =   f4_sim(1,1+floor(time_FFD(ind)/Ts));
%     % v_theta
%     z_sim(18,ind-1) =  v_theta(1,1+floor(time_FFD(ind)/Ts));
% % % % theta 
%      z_sim(17,ind-1) =  theta_sim(1,1+floor(time_FFD(ind)/Ts));


end








x_sim       =   z_sim(1,1:Nblock:end)';
y_sim       =   z_sim(2,1:Nblock:end)';

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


%% lag cost 


    % costo diviso in errore longitudinale e contour
e_c_y = (y_des-y_sim)'*(y_des-y_sim); % errore lungo l'asse x
e_c_x = (x_des - x_sim)'*(x_des- x_sim); % errore lungo l'asse y
normErroreTot = sqrt(e_c_y+e_c_x); % norma del vettore
vetErroreTot = [e_c_x,e_c_y]'; % sono le componenti x y dell'errore
nd = length (x_des);

vetErroreContour = zeros(2,Np);


for i = 1 : nd 

   %tx = derivata di x_d w.r.t to theta sim

tx = df_x(i,1);
ty = df_y(i,1);
matriceTang = [1-tx^2, - tx*ty; 
             -tx*ty, 1 - ty^2];

if (isnan(matriceTang))
matriceTang = zeros(2);
end


% calcolare le componenti del vettore contourn
x_vettore_tot = (x_des(i,1)-x_sim(i,1))'*(x_des(i,1)-x_sim(i,1));
y_vettore_tot = (y_des(i,1)-y_sim(i,1))'*(y_des(i,1)-y_sim(i,1));
xy_vettore_tot = [x_vettore_tot,y_vettore_tot];
vetErroreContour(:,i) =  matriceTang * xy_vettore_tot' ;


end


weight1_din_x = weight1_din(:,1);
weight1_din_y = weight1_din(:,2);
vetErroreContour_x = vetErroreContour(1,:);
vetErroreContour_y = vetErroreContour(2,:);

matrix_weight1_din_x= diag(weight1_din_x');
matrix_weight1_din_y= diag(weight1_din_y');


cost2 = vetErroreContour_x * matrix_weight1_din_x *vetErroreContour_x'...
    + vetErroreContour_y * matrix_weight1_din_y *vetErroreContour_y';

%% definizione errore lag con la differenza
errorLag = vetErroreTot - vetErroreContour;
cost1 = norm(errorLag)^2; 


%% weight ec che varia sencondo le way points: 


%% delta angular speeds cost 
cost3 = p_sim'*p_sim + q_sim'*q_sim + r_sim'*r_sim ; % data da tutti e tre

diff_p = p_sim(2:end,1)-p_sim(1:end-1,1);
diff_q =  q_sim(2:end,1)-q_sim(1:end-1,1);
diff_r =  r_sim(2:end,1)-r_sim(1:end-1,1);
% cost3 = diff_p'*diff_p + diff_q'*diff_q +diff_r'*diff_r ;


%% speed progress  cost
cost4 = u_in(5,:)*u_in(5,:)';
diff_u_in_5 = u_in(5,2:end) -u_in(5,1:end-1);
% cost4 = diff_u_in_5'*diff_u_in_5;


%% speed f1,f2,f3,f4 cost
cost5 = u_in(1,:)*u_in(1,:)'+u_in(2,:)*u_in(2,:)'+u_in(3,:)*u_in(3,:)'+u_in(4,:)*u_in(4,:)';
diff_u_in_1_4 = u_in(1:4,2:end) -u_in(1:4,1:end-1);
% cost5 = diff_u_in_1_4'*diff_u_in_1_4;

%speed progress cost
mu = 1;
cost6 = - mu *sum(progress_speed_sim,'all'); 


%% costo sulla non troppo differenza per il progress_sim
% cost7
% diff_progress_sim = progress_sim(2:end,1)-progress_sim(1:end-1,1);
% cost7= diff_progress_sim'*diff_progress_sim;



cost_tot = weight1*cost1 +weight2*cost2+ weight3*cost3 +weight4* cost4 ...
    +weight5* cost5 +weight6* cost6 ;% +weight7*cost7;
q_h          =   cost_tot;
end

