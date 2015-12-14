% function to fit a quadratic surface to a neighborhood of points

function parameterVector = fitQuadric(neighborHood, numberOfNeighborhoodPoints)

    % M = sum(l(xi) * l(xi)T)
    M = zeros(10, 10);

    for i  = 1 : 1 : numberOfNeighborhoodPoints
   
        % l(x) = (x1^2, x2^2, x3^2, x1*x2, x1*x3, x2*x3, x1, x2, x3, 1)T
        l = [   
                neighborHood(i, 1) ^ 2;
                neighborHood(i, 2) ^ 2;
                neighborHood(i, 3) ^ 2;
                neighborHood(i, 1) * neighborHood(i, 2);
                neighborHood(i, 1) * neighborHood(i, 3);
                neighborHood(i, 2) * neighborHood(i, 3);
                neighborHood(i, 1);
                neighborHood(i, 2);
                neighborHood(i, 3);
                1
            ];
        
        M = M + l * l';
   
    end

    % N = sum(lx(xi) * lx(xi)T) + sum(ly(xi) * ly(xi)T) + sum(lz(xi) * lz(xi)T)
    N = zeros(10, 10);

    for i  = 1: 1 : numberOfNeighborhoodPoints
   
        l1 = [  
                neighborHood(i, 1) * 2;
                0;
                0;
                neighborHood(i, 2);
                neighborHood(i, 3);
                0;
                1;
                0;
                0;
                0
             ];
        
        N = N + l1 * l1';
   
        l2 = [  
                0;
                neighborHood(i, 2) * 2;
                0;
                neighborHood(i, 1);
                0;
                neighborHood(i, 3);
                0;
                1;
                0;
                0
             ];
        
        N = N + l2 * l2';
    
        l3 = [  
                0;
                0;
                neighborHood(i, 3) * 2;
                0;
                neighborHood(i, 1);
                neighborHood(i, 2);
                0;
                0;
                1;
                0
             ];
        
        N = N + l3 * l3';

    end

    % the eigenvector corresponding to the smallest eigenvalue provides the 
    % best-fit parameter vector
    [coefficientEigenVectors, coefficientEigenValues] = eig(M, N);

    coefficientEigenValues = diag(coefficientEigenValues);
    coefficientEigenValues = coefficientEigenValues';

    [~, indexMinCoefficientEigenValue] = min(coefficientEigenValues);

    parameterVector = coefficientEigenVectors(:, indexMinCoefficientEigenValue);
    parameterVector = parameterVector';

end