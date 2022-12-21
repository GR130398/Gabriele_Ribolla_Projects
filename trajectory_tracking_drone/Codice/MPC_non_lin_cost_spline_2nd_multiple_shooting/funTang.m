function [f,gradf] = funTang(x,c0,c1,c2,c3)
% ROSEN computes the Rosenbrock function f(x)= c0 + c1*x+ c2*x^2+c3*x^3, as
% well as its gradient and Hessian at the given point x

f           =  c0 + c1*x+ c2*x^2+c3*x^3 ;

gradf       =   c1+2*c2*x+ 3*c3*x^2;
             
%Hessf       =   [2-4*b*x(2,1)+12*b*x(1,1)^2 -4*b*x(1,1);
               %  -4*b*x(1,1) 2*b];
end
