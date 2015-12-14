% function to ensure there is a gap outside the neighborhood that is wide
% enough to let robot hand to fit in

function gapClearance = hasClearance (neighborHood, numberOfNeighborhoodPoints, principalAxis, centroid, circleRadius, extent)

    % min number of points required to be within the inner cylinder
    minPointsInner = 40;
    % threshold below which the gap is considered to be large enough
    gapThreshold = 5;

    croppedPoints = zeros(numberOfNeighborhoodPoints, 3);
    numberCroppedPoints = 0;
    for i = 1 : 1 : numberOfNeighborhoodPoints
   
        cropped = neighborHood(i, :);
    
        axialDistance = dot(principalAxis, cropped - centroid);
    
        if abs(axialDistance) < extent / 2;
    
            numberCroppedPoints = numberCroppedPoints + 1;
            croppedPoints(i, :) = neighborHood(i, :);

        end

    end

    croppedPoints = croppedPoints(1 : numberCroppedPoints, :);

    identityMatrix = eye(3);

    normalDifference = (identityMatrix - principalAxis' * principalAxis) * (croppedPoints' - repmat(centroid', 1, numberCroppedPoints));

    normalDistance = sqrt(sum(normalDifference.^2));


    % need max hand aperture
    maxHandAperture = 0.093;
    % width of the gap to let robot hand to fit in
    handleGap = 0.20;
    
    % linear search for the outer shell
    for radius = circleRadius : 0.001 : maxHandAperture
    
        numberOfPointsInGap = sum((normalDistance > radius) & (normalDistance < (radius + handleGap)));
       
        numberOfPointsInside = sum(normalDistance <= radius);
    
        if ((numberOfPointsInGap < gapThreshold) && (numberOfPointsInside > minPointsInner));
   
            gapClearance = true;
        
        else
        
            gapClearance = false;
    
        end

    end

end