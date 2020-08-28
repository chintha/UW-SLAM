function [worldPoints, imagePoints] = helperFind3Dto2DCorrespondences(vSet,KLT_currPoints,RefinedXYZ,imagePointsAll,validPointindex, keyPtId)

points2 = vSet.Views.Points{keyPtId};
[~, ia, ib] = intersect(points2,imagePointsAll,'stable','rows');

[~, ic, ~] = intersect(ia,validPointindex,'stable','rows');
ia=ia(ic);
ib=ib(ic);
worldPoints=RefinedXYZ(ib,:);% 3D points common to all three views
imagePoints = double(KLT_currPoints(ia).Location);% 2D points common to all three views % this is OK, reviwed 2nd time

