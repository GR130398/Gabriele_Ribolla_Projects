function [z_sim] = quad_lin_traj_v1(x,xRef2s,yRef2s,th,z0,Ts,Np)
%QUAD_TRAJ Summary of this function goes here
%   Detailed explanation goes here

%% Build vector of inputs

nz = 18;
t_in        =   [0:Ts:(Np-1)*Ts]';

u_in(1,:)=x(1:Np,1)';
u_in(2,:)= x(Np+1:2*Np,1)';
u_in(3,:)= x(2*Np+1:3*Np,1)';
u_in(4,:)= x(3*Np+1:4*Np,1)';
u_in(5,:)= x(4*Np+1:5*Np,1)';


%% Run simulation with FFD
time_FFD    =   [0:0.01:(Np-1)*Ts];
Nblock      =   Ts/0.01;
Nsim_FFD    =   length(time_FFD);

z_sim      =   zeros(nz,Nsim_FFD);
z_sim(:,1) =   z0;

for ind=2:Nsim_FFD
    u                  =   u_in(:,1+floor(time_FFD(ind)/Ts));

  zdot               =   linquadcopter_augmented_v1(0,z_sim(:,ind-1),u,th);
    z_sim(:,ind)       =   z_sim(:,ind-1)+Ts/Nblock*zdot;
end



%% Plot (X,Y) trajectory and constraints

X_sim       =   z_sim(1,1:end);
Y_sim       =   z_sim(2,1:end);

figure(6),plot(Y_sim,X_sim,'b.-',...
    yRef2s,xRef2s,'r.-'); ...
    grid on, hold on,xlabel('x'),ylabel('y')
legend('sim','circuit')
hold off

end
