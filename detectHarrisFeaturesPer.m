function [currPoints]= detectHarrisFeaturesPer(I,roi,numPoints,KLT_currPoints)
% this function detect New harris points and removes points that are
% identicaall and very closer to previous points
validPtNum=length(KLT_currPoints);
currpointsALL= detectHarrisFeatures(I, 'MinQuality' ,0,'FilterSize', 3, 'ROI', roi);
currpoints= currpointsALL.selectStrongest(10*(numPoints-validPtNum));
idx=logical(1:length(currpoints))';
A=double(KLT_currPoints.Location);
B=double(currpoints.Location);
for k=1:4    
    FF = scatteredInterpolant(B(:,1), B(:,2), (1:size(B,1)).', 'nearest');
    BidxNear=FF(A);  
    idx(BidxNear)=false;
    if sum(idx)<numPoints-validPtNum    
        break;
    end
    currpoints=currpoints(idx);
    B=double(currpoints.Location);
    idx=logical(1:length(currpoints))';    
end    
detPoints = selectUniform(currpoints,(numPoints-validPtNum), size(I));
currPoints=[KLT_currPoints;detPoints]; %Rearranging points
end