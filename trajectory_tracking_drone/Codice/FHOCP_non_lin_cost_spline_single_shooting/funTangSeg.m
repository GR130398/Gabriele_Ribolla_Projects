function [tangSeg] = funTangSeg(p1,p2)
%FUNTANGSEG  calcolo tangente di un segmento che connette due punti p1, p2
%   Detailed explanation goes here
x1 = p1(1,1);
x2 = p2(1,1);
y1 = p1(1,2);
y2 = p2(1,2);
%norm1= sqrt((y2-y1)^2 + (x2-x1)^2);7
norm1 = norm(p2-p1,2);
tangSeg(1,1) =  (x2-x1)/norm1;
tangSeg(1,2) =  (y2-y1)/norm1;
end

