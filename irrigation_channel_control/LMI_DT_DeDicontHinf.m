function [K,rho,feas,norm]=LMI_DT_DeDicontHinf(Ftot,Gdec,Hdec,Htot,N,ContStruc,Gw,Dw,~)
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
% - normInfTFnoisesOutput: define the inf norm of the transfert function
%   between the real/fictitious noises/errors and the system ouptuts
% -GW = real/fictitious noises/error matrix in the state repr

% Output:
% - K: structured control gain
% - rho: spectral radius of matrix (F+G*K) - note that [Hdec{1}',...,
% Hdec{N}']=I
% - feas: feasibility of the LMI problem (=0 if yes)

Gtot=[];
%Htot = [];
for i=1:N
    m(i)=size(Gdec{i},2); % input dimension vector for each channel G_i = n x nu_i
    n(i)=size(Hdec{i},1); % output dimension vector
    Gtot=[Gtot,Gdec{i}]; 
 %   Htot =[Htot,Hdec{i}];
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
nw = size(Gw(1,:),2); %columns of gw ==> noises
%mw = size(Dw(1,:)); % number of noises on the output
% without considering noises on the outputs dw = 0 and D = 0; 
Du= zeros(ntot,N); 
normInfTFnoisesOutput = sdpvar(1);
LMIconstr=[[P, Ftot*P+Gtot*L, Gw, zeros(ntot,ntot);
            P*Ftot'+L'*Gtot', P, zeros(ntot,nw), P*Htot'+L'*Du',;
            Gw', zeros(nw,ntot), normInfTFnoisesOutput*eye(nw,nw), Dw';
            zeros(ntot,ntot), Htot*P+Du*L, Dw, normInfTFnoisesOutput*eye(ntot,ntot)]>=1e-6*ones(ntot+ntot+ mtot+ ntot,ntot+ntot+ mtot+ ntot)];
        %zeros(mtot,ntot) H*P+D*L zeros(mtot,nw) normInfTFnoisesOutput*eye(mtot,mtot)]>=1e-2*eye(ntot+ntot+ mtot+ nw,ntot+ntot+ mtot+ nw)];

        %options=sdpsettings('solver ',' mosek');

%J = optimize([Model, -10000 <= allvariables(Model,Objective) <= 10000],Objective)
%J=optimize(LMIconstr,normInfTFnoisesOutput,[],options);
J=optimize(LMIconstr,normInfTFnoisesOutput);%,[],options);
feas=J.problem;
L=double(L);
P=double(P);
norm = double(normInfTFnoisesOutput);
K=L/P;
rho=max(abs(eig(Ftot+Gtot*K)));