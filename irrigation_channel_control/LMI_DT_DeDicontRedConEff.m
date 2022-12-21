function [K,rho,feas,norm]=LMI_DT_DeDicontRedConEff(Ftot,Gdec,Hdec,N,ContStruc,aL,aP)
% Computes, using LMIs, the distributed "state feedback" control law for the discrete-time system, with reference to the control
% information structure specified by 'ContStruc'.
%
% Inputs:
% - Ftot: system matrix.
% - Gdec: input matrices (i.e., Gdec{1},..., Gdec{N} are the input matrices of the decomposed system, one for each channel).
% - Hdec: output matrices  (i.e., Hdec{1},..., Hdec{N} are the output matrices of the decomposed system, one for each channel, where [Hdec{1}',...,
% Hdec{N}']=I).
% - N: number of subsystems.
% - ContStruc: NxN matrix that specifies the information structure
% constraints (ContStruc(i,j)=1 if communication is allowed between channel
% j to channel i, ContStruc(i,j)=0 otherwise).
%
% Output:
% - K: structured control gain
% - rho: spectral radius of matrix (F+G*K) - note that [Hdec{1}',...,
% Hdec{N}']=I
% - feas: feasibility of the LMI problem (=0 if yes)

Gtot=[];

for i=1:N
    m(i)=size(Gdec{i},2); % input dimension vector for each channel G_i = n x nu_i
    n(i)=size(Hdec{i},1); % output dimension vector
    Gtot=[Gtot,Gdec{i}]; 
end
ntot=size(Ftot,1);
mtot=sum(m);

yalmip clear

if ContStruc==ones(N,N)
    % Centralized design
    P=sdpvar(ntot);
    L=sdpvar(mtot,ntot);
else
    % Dentralized/distributed design
    P=[];
    L=sdpvar(mtot,ntot);
    minc=0;
    for i=1:N
        P=blkdiag(P,sdpvar(n(i)));
        ninc=0;
        for j=1:N
            if ContStruc(i,j)==0
                L(minc+1:minc+m(i),ninc+1:ninc+n(j))=zeros(m(i),n(j));
            end
            ninc=ninc+n(j);
        end
        minc=minc+m(i);
    end
end
%Y = 1\P;
kL = sdpvar(1);
kP = sdpvar(1);

LMIconstr=[[P-Ftot*P*Ftot'-Ftot*L'*Gtot'-Gtot*L*Ftot' Gtot*L; L'*Gtot' P]>=1e-6*ones(ntot*2)];
LMIconstr = LMIconstr +  [[kL*eye(mtot,mtot) L; L' eye(ntot)]>=1e-6*ones(mtot+ntot,ntot+mtot)];
LMIconstr = LMIconstr + [[kP*eye(ntot) eye(ntot); eye(ntot) P]>=1e-6*ones(2*ntot)];

%options=sdpsettings('solver','sedumi');
cost = aL*kL + aP*kP; 
J=optimize(LMIconstr,cost);%,[],options);

feas=J.problem;
L=double(L);
P=double(P);
kL = double(kL);
kP=double(kP);
norm = sqrt(kL)*kP;
K=L/P;
rho=max(abs(eig(Ftot+Gtot*K)));