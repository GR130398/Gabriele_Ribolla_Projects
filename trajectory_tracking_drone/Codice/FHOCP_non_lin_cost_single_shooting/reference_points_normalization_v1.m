function [xRef2s,yRef2s,C,gradbp]=reference_points_normalization_v1(tot_punti)


%% load the scene data files
% load data from %Las Vegas Motor Speedway - Outside Road Course - North Variant 

dataTrack = load('LVMS_ORC_NV.mat'); 



%% define reference points

xRef = dataTrack.Inside(:,[1]);
yRef=  dataTrack.Inside(:,[2]);
refPose(:,1) = xRef;
refPose(:,2) = yRef;


%distance = sqrt(xRef'*xRef-yRef'*yRef);
%% calculate distance vector
distancematrix = squareform(pdist(refPose));
distancesteps = zeros(length(refPose)-1,1);
for i = 2:length(refPose)
    distancesteps(i-1,1) = distancematrix(i,i-1);
end
totalDistance = sum(distancesteps); % Total traveled distance
distbp = cumsum([0; distancesteps]); % Distance for each waypoint
gradbp = linspace(0,totalDistance,tot_punti); % Linearize distance

%% linearize X and Y vectors based on distance
xRef2 = interp1(distbp,xRef,gradbp,'pchip');
yRef2 = interp1(distbp,yRef,gradbp,'pchip');
yRef2s = smooth(gradbp,yRef2);
xRef2s = smooth(gradbp,xRef2);

%% plot the circuit
% figure(5),plot(yRef2s,xRef2s,'b.-',yRef,xRef,'r.-',yRef2,xRef2,'g.-'),grid on, hold on,xlabel('y'),ylabel('x')
% legend('ref2s','ref','ref2')
% hold off


%{
For the MATLAB function: spline, use the syntax: pp = spline(x,y)
 to get a structure data. Then extract the coefficients of the cubic 
spline arranged per row in a matrix form: C = pp.coefs, where C rows are:
 [a b c d] of the equation: f(x) = (a(x-x1)^3)+(b(x-x1)^2)+(c(x-x1))+(d)
 in the interval: [x1 x2].

%}
PP = spline(xRef2s,yRef2s);
C = PP.coefs;

%f(x) = (a(x-x 1)^3)+(b(x-x1)^2)+(c(x-x1))+(d);
% %% Curvature Function


%% da uncommentare se si vuole far mettere il primo punto nell'origine
% xRef2s = xRef2s - xRef2s(1);
% yRef2s = yRef2s - yRef2s(1);

end
function curvature = getCurvature(xRef,yRef)
% Calculate gradient by the gradient of the X and Y vectors
DX = gradient(xRef);
D2X = gradient(DX);
DY = gradient(yRef);
D2Y = gradient(DY);
curvature = (DX.*D2Y - DY.*D2X) ./(DX.^2+DY.^2).^(3/2);
end
