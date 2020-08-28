
function [orientation, location, inlierIdx,CEM,EPPfail] = ...
    helperEstimateRelativePose(matchedPoints1, matchedPoints2, cameraParams)
mem=-1;
EPPfail=false;
if ~isnumeric(matchedPoints1)
    matchedPoints1 = matchedPoints1.Location;
end

if ~isnumeric(matchedPoints2)
    matchedPoints2 = matchedPoints2.Location;
end
CEM=true;
for i = 1:200
    % Estimate the essential matrix.    
    [E, inlierIdx,status] = estimateEssentialMatrix(matchedPoints1, matchedPoints2,...
        cameraParams,'Confidence',99,'MaxNumTrials',200,'MaxDistance' ,0.5);
            if status~=0
                EPPfail=true;
                orientation=NaN;
                location=NaN;
                inlierIdx=NaN;
                return;
            end
%estimates the essential matrix from corresponding points in a pair of images using the M-estimator SAmple
   %Consensus (MSAC) algorithm.
    % Make sure we get enough inliers
    
    if sum(inlierIdx) / numel(inlierIdx) < .3% originally this was .3
        continue;
    end
    
    % Get the epipolar inliers.
    inlierPoints1 = matchedPoints1(inlierIdx, :);
    inlierPoints2 = matchedPoints2(inlierIdx, :);    
    
    % Compute the camera pose from the fundamental matrix. Use half of the
    % points to reduce computation.
    [orientation, location, validPointFraction] = ...
        relativeCameraPose(E, cameraParams, inlierPoints1(1:2:end, :),...
        inlierPoints2(1:2:end, :));

    % validPointFraction is the fraction of inlier points that project in
    % front of both cameras. If the this fraction is too small, then the
    % fundamental matrix is likely to be incorrect.
    if validPointFraction > 0.6
       return;
    end
    
    if mem<validPointFraction
        mem= validPointFraction;
        Ori= orientation;
        Loc=location; 
        Inl=inlierIdx;
    end
end
% After 200 attempts validPointFraction is still too low and best match
% will send out
CEM=false;
orientation=Ori;
location=Loc;
inlierIdx=Inl;
clear Ori;
clear Loc;
clear Inl;
end
