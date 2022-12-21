% Irrigation channel


%{
%versione finale da portare all'orale

%TO DO
%}



clear all 
clc 
close all 

grafici = 1;

N=5;
tauS=4; % min - sampling time
tau=[8 4 16 16 16]; % min - delays in continuous-time
k=tau./tauS; % delays in continuous-time
alpha=[22414,11942,43806,43806,43806]; %m^2
% remark: inputs are expressed in m^3/min

 %simulation data for the system trajectories in Discrete time
 x0=[];   % random intial conditions
 Tfinal = 100; % total time of simulation, 100 could be a good time to 
               %selected, because the settling time (time at which the free 
               %motion may svanish ) is more or less 
               %given by 5*tauS = 5*4 = 20
 Ts= tauS;     %sampling time, done because after there is a notation ....
               %that uses Ts against tauS.
 K = 0:Ts:(Tfinal-Ts); % vector for the discrete time steps 
                        %K = 1:Ts:Tfinal; % other possibility 
 
 % to define the random value of the initial state condition
for i=1:N    
    x0=[x0;10*randn(4,1)];
end 

F=[];
G=[];
for i=1:5
    Gc{i}=[];
    for j=1:5
        if j==i
            Delay{i}=zeros(k(i),k(i));
            if k(i)>=2
                Delay{i}(1:end-1,2:end)=eye(k(i)-1);
            end  
            Fc{i}=blkdiag([1],Delay{i});
            Fc{i}(1,2)=1;
            Gc{i}=[Gc{i};[zeros(k(i),1);tauS/alpha(i)]];
        elseif j==i-1
            Gc{i}=[Gc{i};[-tauS/alpha(j);zeros(k(j),1)]];
        else
            Gc{i}=[Gc{i};[zeros(k(j)+1,1)]];
        end
    end
    F=blkdiag(F,Fc{i});
    G=[G,Gc{i}];
end

H=eye(size(F,2));
ntot=size(F,1);
%% state and inputs vectors into subvectors (decomposed system)
ind_h = 1;
for i=1:N
    Gdec{i}=G(:,i);
    Hdec{i}=H(ind_h: ind_h + k(i),:); 
    ind_h=ind_h + k(i);               %it helps to get the states : 1,4,6,11,16
                                      %that reprensents the water levels of
                                      %each channel (subsystem)
end

%% eigenvalues and spectral radius for the open loop system
eig_f_Dt_OL = eig(F);
rhoSpectral_radius_DT_OL= max(abs(eig(F))); %it is not asymptotically stable ...
                                         % because the spectral radius is =
                                         % 1;(it must be < 1, due the fact 
                                         %that the system is in discrete time)
                                         
poly_F = poly(F);
roots_poly_F = roots(poly(F));
%% define the fixed modes DT using the MatLab functions di_fixed_modes and LMI_DT_DeDicont
% and different criteria to compute the control laws Hinf and radisu in a
% disk
%find the fixed modes and control gains for the
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % centralized case for the system
Ftot = F ;
Gtot = G ; 
rounding_n =3;
radius = 0.4; % to have a module lower than 1 to have asynt stab
             % mi permette di avere considerando anche altri valori del
             % radius di avere la stabilità in tutte le configurazioni
             % considerate ed avere una variazione veloce
             % scendendo con il valore avrò una risposta più veloce ma non 
             %la stabilità per tutti i tipi di strutture di constrollo
             %se lo alzo la risposta sarà più lenta. 

center = 0;
aL = 10;
aP = 0.001;   %values normally used
kP= 10;  %for norm of P^-1 lower than kP ==> P lower than 1/kp
kL = 10; % for norm of L lower than kL
normMaxEq = sqrt(kL)*kP; % from the relation of the reduction effort
% L= kx * P ==> Kx = L/P = sqrt(kL)*kP

nu = size(G,2);
Du= ones(ntot,nu);
Gw = ones(ntot,nu);
%Gw = zeros(ntot,N);
Dw = ones(ntot,nu);

normInfTFnoisesOutput = 0.1;
ContStruc_cen=ones(N,N);
[fm_cen_DT,KnoFmCen]=di_fixed_modesV1(Ftot,Gdec,Hdec,N,ContStruc_cen,rounding_n);
[Kas_c_,rhoas_c,feasAs_c]=LMI_DT_DeDicont(Ftot,Gdec,Hdec,N,ContStruc_cen);
[KdiskInAcent,rhoDiskRadiusCent,feasDiskRadiusCent]=LMI_DT_DeDicontDiskInA(Ftot,Gdec,Hdec,N,ContStruc_cen,radius,center);
[KredConEffCent,rhoRedContEffCent,feasRedContEffCent,normKxRedContEffCent]=LMI_DT_DeDicontRedConEff(Ftot,Gdec,Hdec,N,ContStruc_cen,aL,aP);
[KhInfCent,rhohInfcent,feashInfCent,normInfCent]=LMI_DT_DeDicontHinf(Ftot,Gdec,Hdec,H,N,ContStruc_cen,Gw,Dw);
normKredConEffCent = norm(KredConEffCent);

%[Kh2Cent,rhoh2Cent,feash2Cent,normh2Cent]=LMI_DT_DeDicontH2(Ftot,Gdec,Hdec,H,N,ContStruc_cen,Gw,Dw,Du);

maxRealeigKdiskInacent = max(real(eig(Ftot+Gtot*KdiskInAcent)));
maxImeigKdiskInacent =max(imag(eig(Ftot+Gtot*KdiskInAcent)));
%[KrhoNoFmCen]=KnoLMIbutRhoNoFm(Ftot,Gdec,Hdec,N,ContStruc_cen,rounding_n);
%[KrhoCen]=KnoLMIbutRho(Ftot,Gdec,Hdec,N,ContStruc_cen,rounding_n);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   
% decentralized control
ContStruc_decen=diag(ones(N,1));
[fm_Dec,KnoFmDec]=di_fixed_modesV1(Ftot,Gdec,Hdec,N,ContStruc_decen,rounding_n);
[Kas_De,rhoAS_De,feasAs_De]=LMI_DT_DeDicont(Ftot,Gdec,Hdec,N,ContStruc_decen);
[KdiskInAdecen,rhoDiskRadiusdecen,feasDiskRadiusdecen]=LMI_DT_DeDicontDiskInA(Ftot,Gdec,Hdec,N,ContStruc_decen,radius,center);
[KredConEffdecen,rhoRedContEffdecen,feasRedContEffdecen,normKxRedContEffdecen]=LMI_DT_DeDicontRedConEff(Ftot,Gdec,Hdec,N,ContStruc_decen,aL,aP);
[KhInfDecen,rhohInfDecen,feashInfDecen,normInfDecen]=LMI_DT_DeDicontHinf(Ftot,Gdec,Hdec,H,N,ContStruc_decen,Gw,Dw);    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% distributed cases:
% distributed control (unidirectional string)
ContStruc_string_uni=[1 0 0 0 0; 1 1 0 0 0; 0 1 1 0 0; 0 0 1 1 0; 0 0 0 1 1];
[fm_string_uni,KnoFmStringUni]=di_fixed_modesV1(Ftot,Gdec,Hdec,N,ContStruc_string_uni,rounding_n);                    
[Kas_string_uni,rhoAs_string_uni,feasAs_string_uni]=LMI_DT_DeDicont(Ftot,Gdec,Hdec,N,ContStruc_string_uni);
[KdiskInAstring_uni,rhoDiskRadiusstring_uni,feasDiskRadiusstring_uni]=LMI_DT_DeDicontDiskInA(Ftot,Gdec,Hdec,N,ContStruc_string_uni,radius,center);
[KredConEffstring_uni,rhoRedContEffstring_uni,feasRedContEffstring_uni,normRedContEffstring_uni]=LMI_DT_DeDicontRedConEff(Ftot,Gdec,Hdec,N,ContStruc_string_uni,aL,aP);
[KhInfstring_uni,rhohInfstring_uni,feashInfstring_uni,normInfstring_uni]=LMI_DT_DeDicontHinf(Ftot,Gdec,Hdec,H,N,ContStruc_string_uni,Gw,Dw);    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% distributed control (string bi)
ContStruc_string_bi=eye(N);
for i=1:N-1
    ContStruc_string_bi(i,i+1)=1;
    ContStruc_string_bi(i+1,i)=1;
end





[fm_string_bi,KnoFmStringBi]=di_fixed_modesV1(Ftot,Gdec,Hdec,N,ContStruc_string_bi,3);                    
[Kas_string_bi,rhoAs_string_bi,feasAs_string_bi]=LMI_DT_DeDicont(Ftot,Gdec,Hdec,N,ContStruc_string_bi);
[KdiskInAstring_bi,rhoDiskRadiusstring_bi,feasDiskRadiusstring_bi]=LMI_DT_DeDicontDiskInA(Ftot,Gdec,Hdec,N,ContStruc_string_bi,radius,center);
[KredConEffstring_bi,rhoRedContEffstring_bi,feasRedContEffstring_bi,normKxRedContEffstring_bi]=LMI_DT_DeDicontRedConEff(Ftot,Gdec,Hdec,N,ContStruc_string_bi,aL,aP);
[KhInfstring_bi,rhohInfstring_bi,feashInfstring_bi,normInfstring_bi]=LMI_DT_DeDicontHinf(Ftot,Gdec,Hdec,H,N,ContStruc_string_bi,Gw,Dw);    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% distributed control (star bi)
ContStruc_star_bi=eye(N);
for i=2:N
    ContStruc_star_bi(1,i)=1;
    ContStruc_star_bi(i,1)=1;
end


[fm_star_bi,KnoFmStarBi]=di_fixed_modesV1(Ftot,Gdec,Hdec,N,ContStruc_star_bi,3);
[Kas_star_bi,rhoAs_star_bi_DT,feasAs_star_bi]=LMI_DT_DeDicont(Ftot,Gdec,Hdec,N,ContStruc_star_bi);
[KdiskInAstar_bi,rhoDiskRadiusstar_bi,feasDiskRadiusstar_bi]=LMI_DT_DeDicontDiskInA(Ftot,Gdec,Hdec,N,ContStruc_star_bi,radius,center);
[KredConEffstar_bi,rhoRedContEffstar_bi,feasRedContEffstar_bi,normKxRedContEffstar_bi]=LMI_DT_DeDicontRedConEff(Ftot,Gdec,Hdec,N,ContStruc_star_bi,aL,aP);
[KhInfstar_bi,rhohInfstar_bi,feashInfstar_bi,normInfstar_bi]=LMI_DT_DeDicontHinf(Ftot,Gdec,Hdec,H,N,ContStruc_star_bi,Gw,Dw);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% distributed cycle unidirectional
ContStrucCycleUni=eye(N);
for i = 1: N 
for j = 1 : N 
    if (i == 1)
        ContStrucCycleUni(1,N) = 1;
    end
    if (i==j)
    ContStrucCycleUni(i,j) = 1 ;
    if (j>=2) 
        ContStrucCycleUni(i,j-1) = 1; 
    end
    end
end 
end

[fmCycleUni,KnoFmCycleUni]=di_fixed_modesV1(Ftot,Gdec,Hdec,N,ContStrucCycleUni,3);
[KasCycleUni,rhoAsCycleUni,feasAsCycleUni]=LMI_DT_DeDicont(Ftot,Gdec,Hdec,N,ContStrucCycleUni);
[KdiskInACycleUni,rhoDiskRadiusCycleUni,feasDiskRadiusCycleUni]=LMI_DT_DeDicontDiskInA(Ftot,Gdec,Hdec,N,ContStrucCycleUni,radius,center);
[KredConEffCycleUni,rhoRedContEffCycleUni,feasRedContEffCycleUni,normKxRedContEffCycleUni]=LMI_DT_DeDicontRedConEff(Ftot,Gdec,Hdec,N,ContStrucCycleUni,aL,aP);
[KhInfCycleUni,rhohInfCycleUni,feashInfCycleUni,normInfCycleUni]=LMI_DT_DeDicontHinf(Ftot,Gdec,Hdec,H,N,ContStrucCycleUni,Gw,Dw);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% distributed cycle bidirectional
%{
ContStrucCycleBi=eye(N);
for i = 1: N 
for j = 1 : N 
    if (i == 1)
        ContStrucCycleBi(1,N) = 1;
         ContStrucCycleBi(1,2) = 1;
    end
        if (i==j)
        ContStrucCycleUni(i,j) = 1 ; 
        if(i <= N-1 && i >= 2)
        ContStrucCycleUni(i,j-1) = 1; 
        ContStrucCycleUni(i,j+1) = 1;
        end
        end
    if (i == N)
         ContStrucCycleBi(N,1) = 1;
         ContStrucCycleBi(N,N-1) = 1;
    end
end 
end

 ContStrucCycleBi 
%}


 ContStrucCycleBi= [1 1 0 0 1; 
                    1 1 1 0 0;
                    0 1 1 1 0;
                    0 0 1 1 1;
                    1 0 0 1 1];

[fmCycleBi,KnoFmCycleBi]=di_fixed_modesV1(Ftot,Gdec,Hdec,N,ContStrucCycleBi,3);
[KasCycleBi,rhoAsCycleBi,feasAsCycleBi]=LMI_DT_DeDicont(Ftot,Gdec,Hdec,N,ContStrucCycleBi);
[KdiskInACycleBi,rhoDiskRadiusCycleBi,feasDiskRadiusCycleBi]=LMI_DT_DeDicontDiskInA(Ftot,Gdec,Hdec,N,ContStrucCycleBi,radius,center);
[KredConEffCycleBi,rhoRedContEffCycleBi,feasRedContEfCycleBi,normKxRedContEffCycleBi]=LMI_DT_DeDicontRedConEff(Ftot,Gdec,Hdec,N,ContStrucCycleBi,aL,aP);
[KhInfCycleBi,rhohInfCycleBi,feashInfCycleBi,normInfCycleBi]=LMI_DT_DeDicontHinf(Ftot,Gdec,Hdec,H,N,ContStrucCycleBi,Gw,Dw);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% distributed : star unidirectional with channel 5 as center and with an
% external unidirectional cycle
ContStrucStarUniCenter5ExternalCycleUni = [1 1 0 1 1; 
                                           0 1 1 0 1;
                                           0 0 1 1 1
                                           0 0 0 1 1
                                           0 0 0 0 1];
[fmStarUniCenter5ExternalCycleUni,KnoFmStarUniCenter5ExternalCycleUni]=di_fixed_modesV1(Ftot,Gdec,Hdec,N,ContStrucStarUniCenter5ExternalCycleUni,3);
[KasStarUniCenter5ExternalCycleUni,rhoAsStarUniCenter5ExternalCycleUni,feasAsStarUniCenter5ExternalCycleUni]=LMI_DT_DeDicont(Ftot,Gdec,Hdec,N,ContStrucStarUniCenter5ExternalCycleUni);
[KdiskInAStarUniCenter5ExternalCycleUni,rhoDiskRadiusStarUniCenter5ExternalCycleUni,feasDiskRadiusStarUniCenter5ExternalCycleUni]=LMI_DT_DeDicontDiskInA(Ftot,Gdec,Hdec,N,ContStrucStarUniCenter5ExternalCycleUni,radius,center);
[KredConEffStarUniCenter5ExternalCycleUni,rhoRedContEffStarUniCenter5ExternalCycleUni,feasRedContEfStarUniCenter5ExternalCycleUni,normKxRedContEffStarUniCenter5ExternalCycleUni]=LMI_DT_DeDicontRedConEff(Ftot,Gdec,Hdec,N,ContStrucStarUniCenter5ExternalCycleUni,aL,aP);
[KhInfStarUniCenter5ExternalCycleUni,rhohInfStarUniCenter5ExternalCycleUni,feashInfStarUniCenter5ExternalCycleUni,normInfStarUniCenter5ExternalCycleUni]=LMI_DT_DeDicontHinf(Ftot,Gdec,Hdec,H,N,ContStrucStarUniCenter5ExternalCycleUni,Gw,Dw);


%clc
%{
disp('Results (Discrete-time):')
disp(['-  Centralized: Feasibility=',num2str(feas_c_DT),', rho=',num2str(rho_c_DT),', FM=',num2str(fm_cen_DT),'.'])
disp(['-  Decentralized: Feasibility=',num2str(feas_De_DT),', rho=',num2str(rho_De_DT),', FM=',num2str(fm_Dec_DT),'.'])
disp(['-  Distributed (string_uni): Feasibility=',num2str(feas_string_uni_DT),', rho=',num2str(rho_string_uni_DT),', FM=',num2str(fm_string_uni_DT),'.'])
disp(['-  Distributed (string_bi): Feasibility=',num2str(feas_string_bi_DT),', rho=',num2str(rho_string_bi_DT),', FM=',num2str(fm_string_bi_DT),'.'])
disp(['-  Distributed (star_bi): Feasibility=',num2str(feas_star_bi_DT),', rho=',num2str(rho_star_bi_DT),', FM=',num2str(fm_star_bi_DT),'.'])
%}

%grafici =true;
if (grafici)
%%
%%%%%%%%%%%%%
%   Plots   %
%%%%%%%%%%%%%


%--------------------------------------------------------------------------
figure
plot(eig(Ftot+Gtot*KdiskInAcent),'o');
xlabel('real')
ylabel('Im')
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% free state trajectories in open loop (without the control gains defined previously)
for k_=1:Tfinal/Ts
    x_free(:,k_)=((Ftot)^k_)*x0;
end
%final values of the free system trajectory

xFreeOlFinalValues(:,1) = x_free(:,length(K));
ind = 1;

for i=1:N
   xFreeOLFinalValuesChannels(i,1)= x_free(ind,length(K));
    ind = ind + k(i)+ 1; % to show only the state defined by h of that channel
end


figure
ind = 1;
for i=1:N
    subplot(N,1,(i-1)+1)
    hold on
    grid on
    title(['h water level OL chanel_{',num2str(i),'}'])
    plot(K,[x_free(ind,:)],'k.-')
    ind = ind + k(i)+ 1; % to show only the state defined by h of that channel
end
legend('x_freeDT OL')

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% in closed loop only for asymptotic stability 
Gtot = G;
Htot = H ;

for k_=1:Tfinal/Ts
    xAs_c(:,k_)=((Ftot+Gtot*Kas_c_)^k_)*x0;
    xAsDe(:,k_)=((Ftot+Gtot*Kas_De)^k_)*x0;
    xAsString_uni(:,k_)=((Ftot+Gtot*Kas_string_uni)^k_)*x0;
    xAs_string_bi(:,k_)=((Ftot+Gtot*Kas_string_bi)^k_)*x0;
    xAs_star_bi(:,k_)=((Ftot+Gtot*Kas_star_bi)^k_)*x0;
    xAsCycleUni(:,k_)=((Ftot+Gtot*KasCycleUni)^k_)*x0;
    xAsCycleBi(:,k_)=((Ftot+Gtot*KasCycleBi)^k_)*x0;
    xAsStarUniCenter5ExternalCycleUni(:,k_)=((Ftot+Gtot*KasStarUniCenter5ExternalCycleUni)^k_)*x0;

end

ind = 1;

for i=1:N
    xFreeCLfinalCent(i,1) = xAs_c(ind,length(K));
    ind = ind + k(i)+ 1; % to show only the state defined by h of that channel
end

ind = 1;

for i=1:N
   xFreeCLfinalDe(i,1) = xAsDe(ind,length(K));
    ind = ind + k(i)+ 1; % to show only the state defined by h of that channel
end
ind = 1;

for i=1:N
   xFreeCLfinalStringUni(i,1) = xAsString_uni(ind,length(K));
    ind = ind + k(i)+ 1; % to show only the state defined by h of that channel
end
ind = 1;

for i=1:N
    xFreeCLfinalStringBi(i,1) = xAs_string_bi(ind,length(K));
    ind = ind + k(i)+ 1; % to show only the state defined by h of that channel
end

ind = 1;
for i=1:N
   
    xFreeCLfinalStarBi(i,1) = xAs_star_bi(ind,length(K));
    ind = ind + k(i)+ 1; % to show only the state defined by h of that channel
end


figure % in one figure there are all the system trajectories in closed loop
ind = 1;
for i=1:N 
    
    subplot(N,1,(i-1)+1)
    hold on
    grid on
    title(['hWaterLevelCL_{',num2str(i),'}'])
     %[T_in:h:Tfinal]tolto per mettere K 
    plot(K,[xAs_c(ind,:)],'k.-')
    plot(K,[xAsDe(ind,:)],'m.-')
    plot(K,[xAsString_uni(ind,:)],'b.-')
    plot(K,[xAs_string_bi(ind,:)],'g.-')
    plot(K,[xAs_star_bi(ind,:)],'r.-')
    plot(K,[xAsCycleUni(ind,:)],'y.-')
    plot(K,[xAsCycleBi(ind,:)],'c.-')
    plot(K,[xAsStarUniCenter5ExternalCycleUni(ind,:)],'ko')
    %axis([0 T(end) min(x0)-20 max(x0)+20])
    ind = ind + k(i)+ 1;
end
legend('Centralized','Decentralized','Distributed (string unidirectional)'...
       ,'Distributed (string bidirectional)','distributed (star bi)','distributed (Cycle Uni)'....
       ,'distributed (Cycle Bi)','distribuited (StarUniCenter5ExternalCycleUni)');

   
Gtot = G;
Htot = H ;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%define and plot the system trajectories considering the  k for 
% eingenvalues in a disk of radius r and centered in A
for k_=1:Tfinal/Ts
    xDiskInaCentDT(:,k_)=((Ftot+Gtot*KdiskInAcent)^k_)*x0;
    xKdiskInAdecentDT(:,k_)=((Ftot+Gtot*KdiskInAdecen)^k_)*x0;
    xKdiskInAstringInAstring_uni_DT(:,k_)=((Ftot+Gtot*KdiskInAstring_uni)^k_)*x0;
    xKdiskInAstring_bi_DT(:,k_)=((Ftot+Gtot*KdiskInAstring_bi)^k_)*x0;
    xKdiskInAstar_bi_DT(:,k_)=((Ftot+Gtot*KdiskInAstar_bi)^k_)*x0;
    xKdiskInACycleUni(:,k_)=((Ftot+Gtot*KdiskInACycleUni)^k_)*x0;
    xKdiskInACycleBi(:,k_)=((Ftot+Gtot*KdiskInACycleBi)^k_)*x0;
    xdiskInAStarUniCenter5ExternalCycleUni(:,k_)=((Ftot+Gtot*KdiskInAStarUniCenter5ExternalCycleUni)^k_)*x0;


end


figure % in one figure there are all the system trajectories in closed loop
ind = 1;
for i=1:N 
    
    subplot(N,1,(i-1)+1)
    hold on
    grid on
    title(['hWaterLevelCL_{',num2str(i),'}'])
     %[T_in:h:Tfinal]tolto per mettere K 
    plot(K,[xDiskInaCentDT(ind,:)],'k.-')
    plot(K,[xKdiskInAdecentDT(ind,:)],'m.-')
    plot(K,[xKdiskInAstringInAstring_uni_DT(ind,:)],'b.-')
    plot(K,[xKdiskInAstring_bi_DT(ind,:)],'g.-')
    plot(K,[xKdiskInAstar_bi_DT(ind,:)],'r.-')
    plot(K,[xKdiskInACycleUni(ind,:)],'y.-')
    plot(K,[xKdiskInACycleBi(ind,:)],'c.-')
    plot(K,[xdiskInAStarUniCenter5ExternalCycleUni(ind,:)],'ko');
    %axis([0 T(end) min(x0)-20 max(x0)+20])
    ind = ind + k(i)+ 1;
end
legend('DiskInaCentralized','DiskInaDecentralized','DiskInaDistributed (string unidirectional)'...
       ,'DiskInaDistributed (string bidirectional)','DiskInadistributed (star bi)',....
       'DiskInAdistributed (Cycle Uni)'....
       ,'DiskDistributed (Cycle Bi)', 'distribuited (starUniCenter5ExternalCycleUni)');

   
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
   %define and plot the system trajectories considering the k that reduces
   %the control effort
for k_=1:Tfinal/Ts
    xRedConEffCentDT(:,k_)=((Ftot+Gtot*KredConEffCent)^k_)*x0;
    xRedConEffdecentDT(:,k_)=((Ftot+Gtot*KredConEffdecen)^k_)*x0;
    xRedConEffstringInAstring_uni_DT(:,k_)=((Ftot+Gtot*KredConEffstring_uni)^k_)*x0;
    xRedConEffstring_bi_DT(:,k_)=((Ftot+Gtot*KredConEffstring_bi)^k_)*x0;
    xRedConEffstar_bi_DT(:,k_)=((Ftot+Gtot*KredConEffstar_bi)^k_)*x0;
end


figure % in one figure there are all the system trajectories in closed loop
ind = 1;
for i=1:N 
    
    subplot(N,1,(i-1)+1)
    hold on
    grid on
    title(['hWaterLevelCL_{',num2str(i),'}'])
     %[T_in:h:Tfinal]tolto per mettere K 
    plot(K,[xRedConEffCentDT(ind,:)],'k.-')
    plot(K,[xRedConEffdecentDT(ind,:)],'m.-')
    plot(K,[xRedConEffstringInAstring_uni_DT(ind,:)],'b.-')
    plot(K,[xRedConEffstring_bi_DT(ind,:)],'g.-')
    plot(K,[xRedConEffstar_bi_DT(ind,:)],'r.-')
    %axis([0 T(end) min(x0)-20 max(x0)+20])
    ind = ind + k(i)+ 1;
end
legend('RedConEffCentralized','RedConEffDecentralized','RedConEffDistributed (string unidirectional)'...
       ,'RedConEffDistributed (string bidirectional)','RedConEffdistributed (star bi)')

       Gtot = G;
Htot = H ;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%define and plot the system trajectories considering the control gain Kx
%for H inf

for k_=1:Tfinal/Ts
    xkHinfCent(:,k_)=((Ftot+Gtot*KhInfCent)^k_)*x0;
    xKHinfDecen(:,k_)=((Ftot+Gtot*KhInfDecen)^k_)*x0;
    xHinfStringUni(:,k_)=((Ftot+Gtot*KhInfstring_uni)^k_)*x0;
    xHinfStringBi(:,k_)=((Ftot+Gtot*KhInfstring_bi)^k_)*x0;
    xHinfStarBi(:,k_)=((Ftot+Gtot*KhInfstar_bi)^k_)*x0;
    xHinfCycleUni(:,k_)=((Ftot+Gtot*KhInfCycleUni)^k_)*x0;
    xHinfCycleBi(:,k_)=((Ftot+Gtot*KhInfCycleBi)^k_)*x0;
    xHinfStarUniCenter5ExternalCycleUni(:,k_)=((Ftot+Gtot*KhInfStarUniCenter5ExternalCycleUni)^k_)*x0;
end


figure % in one figure there are all the system trajectories in closed loop
ind = 1;
for i=1:N 
    
    subplot(N,1,(i-1)+1)
    hold on
    grid on
    title(['hWaterLevelCL_{',num2str(i),'}'])
     %[T_in:h:Tfinal]tolto per mettere K 
    plot(K,[xkHinfCent(ind,:)],'k.-')
    plot(K,[xKHinfDecen(ind,:)],'m.-')
    plot(K,[xHinfStringUni(ind,:)],'b.-')
    plot(K,[xHinfStringBi(ind,:)],'g.-')
    plot(K,[ xHinfStarBi(ind,:)],'r.-')
     plot(K,[xHinfCycleUni(ind,:)],'y.-')
      plot(K,[xHinfCycleBi(ind,:)],'c.-')
      plot(K,[ xHinfStarUniCenter5ExternalCycleUni(ind,:)],'ko');
    %axis([0 T(end) min(x0)-20 max(x0)+20])
    ind = ind + k(i)+ 1;
end
legend('HinfCentralized','HinfDecentralized','HinfDistributed (string unidirectional)',...
    'HinfDistributed (string bidirect)','HinfDistribuited (StarBi)',....
       'DiskInAdistributed (Cycle Uni)'....
       ,'DiskDistributed (Cycle Bi)','distribuited (HinfStarUniCenter5ExternalCycleUni)'); 
       

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%define and plot the system trajectories considering the  k for 
% eingenvalues in a disk of radius r and centered in A
%{
for k_=1:Tfinal/Ts
    xKredConEffCent(:,k_)=((Ftot+Gtot*KredConEffCent)^k_)*x0;
  %  x_De_DT(:,k_)=((Ftot+Gtot*K_De_DT)^k_)*x0;
   % x_string_uni_DT(:,k_)=((Ftot+Gtot*K_string_uni_DT)^k_)*x0;
    %x_string_bi_DT(:,k_)=((Ftot+Gtot*K_string_bi_DT)^k_)*x0;
    %x_star_bi_DT(:,k_)=((Ftot+Gtot*K_star_bi_DT)^k_)*x0;
end


figure % in one figure there are all the system trajectories in closed loop
ind = 1;
for i=1:N 
    
    subplot(N,1,(i-1)+1)
    hold on
    grid on
    title(['hWaterLevelCL_{',num2str(i),'}'])
     %[T_in:h:Tfinal]tolto per mettere K 
    plot(K,[xKredConEffCent(ind,:)],'k.-')
   % plot(K,[x_De_DT(ind,:)],'m.-')
  %  plot(K,[x_string_uni_DT(ind,:)],'b.-')
   % plot(K,[x_string_bi_DT(ind,:)],'g.-')
   % plot(K,[x_star_bi_DT(ind,:)],'r.-')
    %axis([0 T(end) min(x0)-20 max(x0)+20])
    ind = ind + k(i)+ 1;
end
legend('RedContEffCentralized')%,'Decentralized','Distributed (string unidirectional)'...
       %,'Distributed (string bidirect
       
       
       

%define and plot the system trajectories considering the  k for hinf 
for k_=1:Tfinal/Ts
   % xKhInfCent(:,k_)=((Ftot+Gtot*KhInfCent)^k_)*x0;
  %  x_De_DT(:,k_)=((Ftot+Gtot*K_De_DT)^k_)*x0;
   % x_string_uni_DT(:,k_)=((Ftot+Gtot*K_string_uni_DT)^k_)*x0;
    %x_string_bi_DT(:,k_)=((Ftot+Gtot*K_string_bi_DT)^k_)*x0;
    %x_star_bi_DT(:,k_)=((Ftot+Gtot*K_star_bi_DT)^k_)*x0;
end


figure % in one figure there are all the system trajectories in closed loop
ind = 1;
for i=1:N 
    
    subplot(N,1,(i-1)+1)
    hold on
    grid on
    title(['hWaterLevelCL_{',num2str(i),'}'])
     %[T_in:h:Tfinal]tolto per mettere K 
   % plot(K,[xKhInfCent(ind,:)],'k.-')
   % plot(K,[x_De_DT(ind,:)],'m.-')
  %  plot(K,[x_string_uni_DT(ind,:)],'b.-')
   % plot(K,[x_string_bi_DT(ind,:)],'g.-')
   % plot(K,[x_star_bi_DT(ind,:)],'r.-')
    %axis([0 T(end) min(x0)-20 max(x0)+20])
    ind = ind + k(i)+ 1;
end
legend('KhInfCent')%,'Decentralized','Distributed (string unidirectional)'...
       %,'Distributed (string bidirect
       
 %}
%{   
 % plot of case with k based on rho 
   %computation of system trajectories in this case
for k_=1:Tfinal/Ts
    x_c_rho_DT(:,k_)=((Ftot+Gtot*KrhoCen)^k_)*x0;
    %x_De_DT(:,k_)=((Ftot+Gtot*K_De_DT)^k_)*x0;
    %x_string_uni_DT(:,k_)=((Ftot+Gtot*K_string_uni_DT)^k_)*x0;
    %x_string_bi_DT(:,k_)=((Ftot+Gtot*K_string_bi_DT)^k_)*x0;
    %x_star_bi_DT(:,k_)=((Ftot+Gtot*K_star_bi_DT)^k_)*x0;
end

   figure % in one figure there are all the system trajectories in closed loop
ind = 1;
for i=1:N 
    
    subplot(N,1,(i-1)+1)
    hold on
    grid on
    title(['hWaterLevelCL_{',num2str(i),'}'])
     %[T_in:h:Tfinal]tolto per mettere K 
    plot(K,[x_c_rho_DT(ind,:)],'k.-')
    %plot(K,[x_De_DT(ind,:)],'m.-')
   % plot(K,[x_string_uni_DT(ind,:)],'b.-')
    %plot(K,[x_string_bi_DT(ind,:)],'g.-')
    %plot(K,[x_star_bi_DT(ind,:)],'r.-')
    %axis([0 T(end) min(x0)-20 max(x0)+20])
    ind = ind + k(i)+ 1;
end
legend('CentralizedRho')%,'Decentralized','Distributed (string unidirectional)'...
    %   ,'Distributed (string bidirectional)','distributed (star bi)')
   
    
   %plots of system strajectories with k based on Rho and no FM

   %computation of system trajectories in this case
for k_=1:Tfinal/Ts
    x_c_rhoNoFm_DT(:,k_)=((Ftot+Gtot*KrhoNoFmCen)^k_)*x0;
    %x_De_DT(:,k_)=((Ftot+Gtot*K_De_DT)^k_)*x0;
    %x_string_uni_DT(:,k_)=((Ftot+Gtot*K_string_uni_DT)^k_)*x0;
    %x_string_bi_DT(:,k_)=((Ftot+Gtot*K_string_bi_DT)^k_)*x0;
    %x_star_bi_DT(:,k_)=((Ftot+Gtot*K_star_bi_DT)^k_)*x0;
end

   figure % in one figure there are all the system trajectories in closed loop
ind = 1;
for i=1:N 
    
    subplot(N,1,(i-1)+1)
    hold on
    grid on
    title(['hWaterLevelCL_{',num2str(i),'}'])
     %[T_in:h:Tfinal]tolto per mettere K 
    plot(K,[x_c_rhoNoFm_DT(ind,:)],'k.-')
    %plot(K,[x_De_DT(ind,:)],'m.-')
   % plot(K,[x_string_uni_DT(ind,:)],'b.-')
    %plot(K,[x_string_bi_DT(ind,:)],'g.-')
    %plot(K,[x_star_bi_DT(ind,:)],'r.-')
    %axis([0 T(end) min(x0)-20 max(x0)+20])
    ind = ind + k(i)+ 1;
end
legend('CentralizedRhonoFm')%,'Decentralized','Distributed (string unidirectional)'...
    %   ,'Distributed (string bidirectional)','distributed (star bi)')
 %}  
   %{
Gprova = zeros(20,5);
systemDT_c_DT= ss(Ftot+Gtot*K_c_DT,Gprova,H,L,Ts);
[Y_forced_step_c_DT,T_step] = step(systemDT_c_DT,t);
Y_forced_input_signal_c_DT = lsim(systemDT_c_DT,u,t);
 
 for i = 1 : N
u(:,i) = 100*sin(t);
end

figure(4)
for i=1:N
    
    subplot(N,1,(i-1)+1)
    hold on
    grid on
    title(['h water level output OL_{',num2str(i),'}'])
     
    plot([0:h:Tfinal],[Y_forced_step_c_DT(:,:,i)],'k.-');
%    plot([h:h:Tfinal],[Y_forced_input_signal_c_DT(:,:,i)],'m.-');
    %plot([h:h:Tfinal],[Y_forced_input_signal(:,ind)],'b.-');

  

end
legend('y_forced_step_ol','y_forced_impulse_ol','y_forced_signal_ol')

   %} 

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% plot all the states to comparison with only the 5 channel 
figure 
n = size(F,1); % number of states
for i=1:n
    subplot(n,1,(i-1)+1)
    hold on
    grid on
    title(['h water level OL_{',num2str(i),'}'])
    plot(K,[x_free(i,:)],'k.-')
end
legend('x_free all states OL')

figure % plot of all states in centralized case
n = size(F,1); % number of states
for i=1:n
    subplot(n,1,(i-1)+1)
    hold on
    grid on
    title(['h water level OL_{',num2str(i),'}'])
    plot(K,[xAs_c(i,:)],'k.-')
end
legend('x_free all states centralized ')


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% plot of forced output to an input signal = 1 constant for t>=0 to 
% test and verify if the system in open loop is non asymptotic stable

% define the discrete time system of the model considering the matrices of
% it 
Ltot = 0;
systemDT = ss(F,Gtot,Htot,Ltot,Ts);
u = zeros(length(K),N);
for i = 1 : N
u(:,i) = 1;
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%forced input signal constat 
Y_forced_input_signal = lsim(systemDT,u,K);

ind = 1;
figure
for i=1:N
    subplot(N,1,(i-1)+1)
    hold on
    grid on
    title(['h_ water_ level _output _ OL _{',num2str(i),'}'])
    plot(K,[Y_forced_input_signal(:,ind)],'b.-');
    
    ind = ind + k(i)+ 1;
end
legend('y_forced_signal')

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%forced step input case
[Y_forced_step,~] = step(systemDT,K);
ind = 1;
figure
for i=1:N
    subplot(N,1,(i-1)+1)
    hold on
    grid on
    title(['h_ water_ level _output _ OL _{',num2str(i),'}'])
    plot(K,[Y_forced_step(:,:,i)],'r.-');
    
    ind = ind + k(i)+ 1;
end
legend('yForcedStep')


       %% closed loop properties 
    xAs_c(:,k_)=((Ftot+Gtot*Kas_c_)^k_)*x0;
    xAsDe(:,k_)=((Ftot+Gtot*Kas_De)^k_)*x0;
    xAsString_uni(:,k_)=((Ftot+Gtot*Kas_string_uni)^k_)*x0;
    xAs_string_bi(:,k_)=((Ftot+Gtot*Kas_string_bi)^k_)*x0;
    xAs_star_bi(:,k_)=((Ftot+Gtot*Kas_star_bi)^k_)*x0;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    
%eigenvalues 
eigenvaluesCentralized = eig(Ftot+Gtot*Kas_c_);
eigenvaluresDecentralized =eig(Ftot+Gtot*Kas_De);
eigenvaluesDistribuitedStringUni = eig(Ftot+Gtot*Kas_string_uni);
eigenvaluesDistribuitedStringBid = eig(Ftot+Gtot*Kas_string_bi);
eigenvaluesDistribuitedStarBid = eig(Ftot+Gtot*Kas_star_bi);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%stability ==> the spectral radius modulus must be lower than 1 
                %to have asymptotic stability
% NOTE they are already given using the LMI_DT_DeDiCont function
% it is possible to say that all the spectral radius are lower than 1 
% so the closed loop system are all aympotically stable in Discrete time
rhoCentralized = max(abs(eigenvaluesCentralized));
rhoDecentralized = max(abs(eigenvaluresDecentralized));
rhoDistribuitedStringBid = max(abs(eigenvaluesDistribuitedStringUni));
rhoDistribuitedStringUni = max(abs(eigenvaluesDistribuitedStringBid));
rhoDistribuitedStarBid = max(abs(eigenvaluesDistribuitedStarBid));

figure % in one figure there are all the system trajectories in closed loop
ind = 1;
for i=1:N 
    
    subplot(N,1,(i-1)+1)
    hold on
    grid on
    title(['hWaterLevelCL_{',num2str(i),'}'])
     %[T_in:h:Tfinal]tolto per mettere K 
    plot(K,[xkHinfCent(ind,:)],'k.-')
    plot(K,[xKHinfDecen(ind,:)],'m.-')
    plot(K,[xHinfStringUni(ind,:)],'g.-')
    plot(K,[xHinfStringBi(ind,:)],'r.-')
   % plot(K,[ xHinfStarBi(ind,:)],'r.-')
    % plot(K,[xHinfCycleUni(ind,:)],'y.-')
    %  plot(K,[xHinfCycleBi(ind,:)],'c.-')
    %  plot(K,[ xHinfStarUniCenter5ExternalCycleUni(ind,:)],'ko');
    %axis([0 T(end) min(x0)-20 max(x0)+20])
    ind = ind + k(i)+ 1;
end
legend('HinfCentralized','HinfDecentralized','HinfDistributed (string unidirectional)','HinfDistributed (string bidirectional)'); 
       
figure % in one figure there are all the system trajectories in closed loop
ind = 1;
for i=1:N 
    
    subplot(N,1,(i-1)+1)
    hold on
    grid on
    title(['hWaterLevelCL_{',num2str(i),'}'])
     %[T_in:h:Tfinal]tolto per mettere K 
    plot(K,[xDiskInaCentDT(ind,:)],'k.-')
    plot(K,[xKdiskInAdecentDT(ind,:)],'m.-')
    plot(K,[xKdiskInAstringInAstring_uni_DT(ind,:)],'g.-')
    plot(K,[xKdiskInAstring_bi_DT(ind,:)],'r.-')
   % plot(K,[xKdiskInAstar_bi_DT(ind,:)],'r.-')
   % plot(K,[xKdiskInACycleUni(ind,:)],'y.-')
   % plot(K,[xKdiskInACycleBi(ind,:)],'c.-')
   % plot(K,[xdiskInAStarUniCenter5ExternalCycleUni(ind,:)],'ko');
    %axis([0 T(end) min(x0)-20 max(x0)+20])
    ind = ind + k(i)+ 1;
end
legend('DiskInaCentralized','DiskInaDecentralized','DiskInaDistributed (string unidirectional)','DiskInaDistributed (string bidirectional)');


figure % in one figure there are all the system trajectories in closed loop
ind = 1;
for i=1:N 
    
    subplot(N,1,(i-1)+1)
    hold on
    grid on
    title(['hWaterLevelCL_{',num2str(i),'}'])
     %[T_in:h:Tfinal]tolto per mettere K 
    plot(K,[xAs_c(ind,:)],'k.-')
    plot(K,[xAsDe(ind,:)],'m.-')
    plot(K,[xAsString_uni(ind,:)],'g.-')
    plot(K,[xAs_string_bi(ind,:)],'r.-')
   % plot(K,[xAs_star_bi(ind,:)],'r.-')
   % plot(K,[xAsCycleUni(ind,:)],'y.-')
   % plot(K,[xAsCycleBi(ind,:)],'c.-')
   % plot(K,[xAsStarUniCenter5ExternalCycleUni(ind,:)],'ko')
    %axis([0 T(end) min(x0)-20 max(x0)+20])
    ind = ind + k(i)+ 1;
end
legend('Centralized','Decentralized','Distributed (string unidirectional)','Distributed (string bidirectional)');

end