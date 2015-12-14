% functon to fit a circle to the points projected on a orthogonal plane to 
% the axis

function [circleCenterX, circleCenterY, circleRadius, shellCentroid, shellExtent] = fitCylinder(neighborHood, numberOfNeighborhoodPoints, normal, principalAxis, neighborhoodCentroid)

    rotationMatrix = zeros(3, 3);

    rotationMatrix(1, :) = normal;
    rotationMatrix(2, :) = principalAxis;
    rotationMatrix(3, :) = cross(normal, principalAxis);

    rotationPoints = neighborHood';
    rotationPoints = rotationMatrix * rotationPoints;
    rotationPoints = rotationPoints';
    
    %rotationPointsBasis = rotationPoints;
    %neighborhoodCentroid = [neighborhoodCentroid(1,1),neighborhoodCentroid(1,2),neighborhoodCentroid(1,3)];
    %neighborhoodCentroid = rotationMatrix * neighborhoodCentroid';
    %neighborhoodCentroid = neighborhoodCentroid';
    
    %for i = 1 : 1 : numberOfNeighborhoodPoints
    
        %rotationPointsBasis(i,2) = neighborhoodCentroid(1,2);
    
    %end
    
    %rotationPointsBasis = rotationMatrix' * rotationPointsBasis';
    %rotationPointsBasis = rotationPointsBasis';
     
    % plot3(rotationPointsBasis(:, 1), rotationPointsBasis(:, 2),rotationPointsBasis(:, 3), 'c.');
    % hold on;

    Q = zeros(3, 3);
    c = zeros(3, 1);
    for i = 1 : 1 : numberOfNeighborhoodPoints
        
        % li = (xi, yi, 1)T
        li = [
                 rotationPoints(i, 1);
                 rotationPoints(i, 3);
                1
             ];
         
        % Q = sum(li)
        Q = Q + li * li';

        % lambdai = xi^2 + yi^2
        lambdai = rotationPoints(i, 1) ^ 2 + rotationPoints(i, 3) ^ 2;
        
        % c = sum(lambdai * li)
        c = c + lambdai * li;
    
    end

    w =  - inv(Q) * c;

    circleCenterX = - 0.5 * w(1);
    circleCenterY = - 0.5 * w(2);
    circleRadius = sqrt(circleCenterX ^ 2 + circleCenterY ^ 2 - w(3));

    axisCoordinate = sum(rotationPoints(:, 2)) / numberOfNeighborhoodPoints;

    centroidCylinderNoRotation = [
                                    circleCenterX; 
                                    axisCoordinate;
                                    circleCenterY
                                 ];
    shellCentroid = rotationMatrix' * centroidCylinderNoRotation;
    shellCentroid = shellCentroid';

    shellExtent = max(rotationPoints(:, 2)) - min(rotationPoints(:, 2));
    
    %visualize circle
    %circleCenter = [circleCenterX, neighborhoodCentroid(1,2), circleCenterY];
    
    %circleCenter = rotationMatrix' * circleCenter';
    %circleCenter = circleCenter';
    
    
    %plot3(circleCenter(1), circleCenter(2), circleCenter(3), 'mo');
    %hold on;
    %plotCircle3D(circleCenter, principalAxis, circleRadius);
    %hold on;
    
end