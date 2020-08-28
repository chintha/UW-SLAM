function [RefinedXYZ,imagePointsAll,camPoses,Reidx]=refinepose(vSet2,viewId,cameraParams,imagePoints,worldPoints)

valid3Dpoint=[];
      fixedIds=[viewId-1];  
      tracks2 = findTracks(vSet2,viewId-1:viewId); % Find point tracks spanning multiple views. 
      camPoses2 = poses(vSet2,viewId-1:viewId);
      
      for i=1:length(tracks2)
       valid3Dpoint(i,:)=tracks2(i).Points(end,:); % all the image points 2ms
      end
      [~, ia, ib] = intersect(valid3Dpoint,imagePoints,'stable','rows');
           
    [~, camPoses,~] = bundleAdjustment(worldPoints(ib,:), tracks2(ia), camPoses2, ...
    cameraParams, 'PointsUndistorted', true, 'AbsoluteTolerance', 1e-9,...
   'RelativeTolerance', 1e-9, 'MaxIterations', 1000, 'FixedViewIDs',fixedIds);

    vSet2 = updateView(vSet2, camPoses);  
 
   %% point Triangulation and BA   
            
            valid3Dpoint=[];
           
            tracks = findTracks(vSet2); % Find point tracks spanning multiple views. 
         
            camPoses = poses(vSet2); 
            idx=logical(1:length(tracks))';
            for p=1:length(tracks)
            L=tracks(p).ViewIds;
                if L(end)~=viewId
                    idx(p)=false;
                end

            end
            [RefinedXYZ, reprojErrors] = triangulateMultiview(tracks(idx), camPoses, cameraParams);
            Reidx = reprojErrors < 1.2;
            
            LOC1=camPoses.Location{end};
            LOC2=camPoses.Location{end-1};
            mid_Point= median(RefinedXYZ(:,3));
            mididx1=abs(RefinedXYZ(:,3))<abs(1.5*mid_Point)+20;
            Reidx=Reidx & mididx1;
            tracks1=tracks(idx);
            if viewId>5
            fixedIds=[viewId-4:viewId-2];
            else
             fixedIds=[viewId-vSet2.NumViews+1+viewId-vSet2.NumViews+2]; 
            end
            
            for i=1:length(tracks)
               valid3Dpoint(i,:)=tracks(i).Points(end,:); % all the image points 2ms
            end
              imagePointsAll=valid3Dpoint(idx,:);
              imagePointsAll=imagePointsAll(Reidx,:); %changed

            RefinedXYZ=RefinedXYZ(Reidx,:);%changed
            [~,ia,ib] = intersect(imagePoints,imagePointsAll,'stable','rows');
            [RefinedXYZ,camPoses,RE_ERROR] = bundleAdjustment(RefinedXYZ, tracks1(Reidx), camPoses, ...
            cameraParams, 'PointsUndistorted', true, 'AbsoluteTolerance', 1e-9,...
            'RelativeTolerance', 1e-9, 'MaxIterations', 10000, 'FixedViewIDs',fixedIds);
            XYZError = RE_ERROR < 1.2;
            RefinedXYZ=RefinedXYZ(XYZError,:);
            imagePointsAll=imagePointsAll(XYZError,:);
            vSet2 = updateView(vSet2, camPoses);  
            [~,reprojErrors] = triangulateMultiview(tracks2, camPoses, cameraParams);
            Reidx = reprojErrors < 1.2;

end