function [f,df,ddf]= spline_f_fd_fdd_v2(x,x_spline_coeff,x1)
%{
For the MATLAB function: spline, use the syntax: pp = spline(x,y)
 to get a structure data. Then extract the coefficients of the cubic 
spline arranged per row in a matrix form: C = pp.coefs, where C rows are:
 [a b c d] of the equation: f(x) = (a(x-x1)^3)+(b(x-x1)^2)+(c(x-x1))+(d)
 in the interval: x on [x1 x2].
%}

% ind = 4;
a = x_spline_coeff(1,1);
b = x_spline_coeff(1,2);
c = x_spline_coeff(1,3);
d = x_spline_coeff(1,4);
% x = gradbp(ind);
% x1 = gradbp(ind-1);
% x2 = gradbp(ind+1);

    f =  (a*(x-x1)^3)+(b*(x-x1)^2)+(c*(x-x1))+(d);
    df = 3*a*(x-x1)^2+ 2*b*(x-x1) + c;
    ddf = 2*3*a*(x-x1) + 2*b;
end