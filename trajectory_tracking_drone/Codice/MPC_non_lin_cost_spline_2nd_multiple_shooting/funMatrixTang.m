function [matrixTang] = funMatrixTang(p2,p1)
% calcolo matrice tangente, tolte le componenti z perchè usiamo solo x y
   [tangSeg] = funTangSeg(p2,p1);
   tx = tangSeg(1,1);
   ty = tangSeg(1,2);
   n = length(p2);
   
   matrixTang = [1-tx^2, - tx*ty; 
                 -tx*ty, 1 - ty^2];
if (isnan(matrixTang))
    matrixTang = zeros(n);
end

end
