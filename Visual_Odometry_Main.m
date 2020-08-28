clear all;
close all;
Re_Tracking=false; % Retracking on/off
enhancement=false;  % Image Enhancement on/off Use with turbit images Specially for Underwater
num_of_images= 360; % Total number of frames need to navigate
viewId = 1;
startkeypoint=false;
Foldername = fullfile('D:\Underwater Nevigation\Data sets and other softwares\UW-VO dataset\datasets\Image seq3'); % Path to image files/ change accordingly
pallOp = gcp(); % Initialized Parallel operations
keypointimages=[];
worldpointData={};
imagepointData={};
Frame_count=1; % Frame Counter
%% setting disparaty timits and other coeficient
Dis_limit=30;% Disparaty Limit
numPoints =1000;
RE=1;
conFI=99.99;
blocksize=[21,21];% Block size for KLT Tracker increase this will robust to occusion,poor on time
C_Vec=(1:numPoints)';
validIdxLogic=false(numPoints,1); % Retracking points store 
validIdxLogic1=false(numPoints,1);% Retracking points store
validIdxLogic2=false(numPoints,1);% Retracking points store
%% set Camera calibration matrix
K=  [257.3408,0,160;0,257.3408,120;0,0,1]';% Change according to dataset
cameraParams = cameraParameters('IntrinsicMatrix', K);
%% Setting Data carriers, trackers
vSet = viewSet;
vSet2 = viewSet;
player = vision.VideoPlayer('Position', [0, 200, 600, 500]);
tracker = vision.PointTracker('MaxBidirectionalError', 0.5, 'NumPyramidLevels',3,'BlockSize',blocksize,'MaxIterations',50); % Initialized KTL Tracker
%% Setting reagion of Interest
startframe = 1;
i=startframe;
I= im2double(imread(fullfile(Foldername, [num2str(i,'%d') '.png'])));
border =30; 
l=size(I, 2)- 2*border;
w=size(I, 1)- 2*border;
roi=[border,border,l,w];
%% Defne first Pose
GTO=[1 0 0;0 -1 0;0 0 -1];  % Grount Thruth Orientation
GTL=[1 1 1]; % Ground Thruth Location
%% Display settings
f1=figure('Position', [605, 200, 600, 500]);
view(gca, 3);
set(gca,'XLim',[-50 50], 'YLim',[-30 130], 'ZLim',[-50 30]);
set(gca, 'CameraUpVector', [0, 1, 0]);
camorbit(gca, -120, 0, 'data', [0, 1, 0]);
axis 'equal';
grid on
xlabel('X (cm)');
ylabel('Y (cm)');
zlabel('Z (cm)');
view(3);
hold on
figure(f1);
camEstimated = plotCamera('Size', 1, 'Location',[0 0 0] , 'Orientation', GTO,'Color', 'r', 'Opacity', 0);
trajectoryEstimated = plot3(0, 0, 0, 'r-','LineWidth',1.5);
title('Camera Trajectory');
ptSHOW=plot3(0, 0, 0, 'o','Color','m','MarkerSize',0.5);
%% Setting first Immage     
I = undistortImage(I,cameraParams);
if enhancement
    I = enhancefun(I);
end
%% detecting Harris coners and initialized Tracker
detPoints=detectHarrisFeatures(I, 'MinQuality' ,0.0,'FilterSize',3, 'ROI', roi);
detPoints=detPoints.selectStrongest(5*numPoints);
detPoints = selectUniform(detPoints, numPoints, size(I));
release (tracker);
initialize(tracker, detPoints.Location, I);
%% display first image
out = insertMarker(I,detPoints.Location,'s','Size',2,'Color','yellow');
player(out);
       
%% setting first view
vSet = addView(vSet, viewId, 'Points', detPoints, 'Orientation', GTO,'Location', GTL);
prevPoints=detPoints;
%% Make first Two poses 
    for q=i+1:num_of_images 
        I = undistortImage(im2double(imread(fullfile(Foldername, [num2str(q,'%d') '.png']))), cameraParams);%
        if enhancement
            I = enhancefun(I);
        end
        [KLT_currPoints, validIdx] = step(tracker, I); %Traking using KTL
        KLT_currPoints=cornerPoints(abs(KLT_currPoints));
        [tform] = estimateGeometricTransform(KLT_currPoints(validIdx).Location,prevPoints(validIdx).Location,...
            'similarity'); % estimating the transform between frames
        tform.T(3,1)=0;
        tform.T(3,2)=0;
        U_tran = transformPointsForward(tform,KLT_currPoints.Location);%% un-rotate the image for accurate disparaty comutation
        disparaty =vecnorm(prevPoints.Location-U_tran,2,2);%0.9ms
        disparaty=median(disparaty(validIdx));
        out = insertMarker(I,KLT_currPoints(validIdx).Location,'s','Size',2,'Color','yellow');
        player(out); % diaplay function
        if disparaty>20 || sum(validIdx)<0.5* numPoints % check for key frame creation
            viewId=viewId+1;
            DHFP= parfeval(pallOp,@detectHarrisFeaturesPer,1,I,roi,numPoints,KLT_currPoints(validIdx)); % detect Harris coners/Parallel OP
            [orient, loc, inlierIdx,~,EPPfail] = helperEstimateRelativePose(...
                prevPoints(validIdx), KLT_currPoints(validIdx), cameraParams);
            matches1= C_Vec(validIdx);
            matches2=(1:sum(validIdx))';
            matchesKeyPt= cat(2,matches1,matches2); % Arranging 2D-2D Correspondences
            matchesKeyPt= matchesKeyPt(inlierIdx,:); % Removing Outliers from Correspondences
            orient_M=orient*GTO; %estimation of Orientation
            loc_M=GTL+loc*GTO; %estimation of Position
            currPoints=fetchOutputs(DHFP); % Getting new Corner points/ this removes identical points and closer points to previous points
            vSet = addView(vSet, viewId, 'Points', currPoints, 'Orientation', orient_M, ...
                'Location',loc_M); % Store camera Poses
            vSet = addConnection(vSet, viewId-1, viewId, 'Matches',matchesKeyPt); % store correspondeces    
            %% this is a workaround to minimize computational delay
            if viewId>3
                vSet2=deleteView(vSet,1:viewId-2);
            else
                vSet2=vSet;
            end
            tracks = findTracks(vSet2,viewId-1:viewId); % Find point tracks spanning multiple views.
            camPoses = poses(vSet2,viewId-1:viewId); % Find Camera Poses
            [worldPoints] = triangulateMultiview(tracks, camPoses, cameraParams); % triangulate Points
            [RefinedXYZ,imagePointsAll,camPoses,Reidx]= refinepose(vSet2,viewId,...
                cameraParams,currPoints(matchesKeyPt(:,2)).Location, worldPoints); % Bundle Adjustmet
            vSet = updateView(vSet, camPoses); % Update Camera poses
            
            RPP=parfeval(pallOp,@refinepose,4,vSet2,viewId,...
                cameraParams,currPoints(matchesKeyPt(:,2)).Location, worldPoints);  % only for fetching perpose for countinue operation                   
    
            helperUpdateCameraPlots(viewId, camEstimated, poses(vSet));
            helperUpdateCameraTrajectories(viewId, trajectoryEstimated,poses(vSet));
            setPoints(tracker, currPoints.Location); % give set function with true tracking points
            keyPtId=viewId;
            %% storing data for visualization
            worldpointData{viewId-1}=RefinedXYZ;
            worldpointData{viewId}=RefinedXYZ;
            break;
        end
        
    end
%% storing 2D 3D correspondence
    
prevPoints   = currPoints;
i=q;
KTL_prevPoints=currPoints.Location;
%% Diaplay Points Cloud
BB=cell2mat(worldpointData');
set(ptSHOW, 'XData', BB(1:20:end,1), 'YData',BB(1:20:end,2), 'ZData', BB(1:20:end,3));
%% main Loop
    while (i < num_of_images)
        
        i=i+1;
        I = undistortImage(im2double(imread(fullfile(Foldername, [num2str(i,'%d') '.png']))), cameraParams);% Getting next Frame
        if enhancement
            I = enhancefun(I); %Image Enhancement
        end
        [KLT_currPoints, validIdx] = step(tracker, I);% KTL Tracker
        KLT_currPoints=abs(KLT_currPoints);
        %% retracking mechanisum
        if Re_Tracking
            validIdxLogic3=validIdxLogic2;
            validIdxLogic2=validIdxLogic1;
            validIdxLogic1=validIdxLogic;
            validIdxLogic= ~validIdx;
            nexttrcIDs=  (validIdxLogic & ~validIdxLogic1 ) | (validIdxLogic1 & ~validIdxLogic2 ) |(validIdxLogic2 & ~validIdxLogic3 ) | validIdx;
        end  
        %% Display function
        out = insertMarker(I,KLT_currPoints(validIdx,:),'s','Size',2,'Color','yellow');
        player(out);
        %% calculating transform and parallex
        if ~isempty(KLT_currPoints(validIdx,:))
            tform = estimateGeometricTransform(KLT_currPoints(validIdx,:),prevPoints(validIdx).Location,...
                'similarity');
            tform.T(3,1)=0;
            tform.T(3,2)=0;
            U_tran = transformPointsForward(tform,KLT_currPoints);            
        else
            disp('Traking loss 1  due to 2D point loss...');
        end  
        disparaty =vecnorm(prevPoints.Location- U_tran,2,2);
        disparatyFrames =vecnorm(KTL_prevPoints(validIdx,:)-KLT_currPoints(validIdx,:),2,2);
        disparatyFrames= mean(disparatyFrames);
        disparaty=mean(disparaty(validIdx));        
        KTL_prevPoints=KLT_currPoints;
        %% avoiding stationary situations(skipping frames)
        if disparatyFrames<0.5 && sum(validIdx)<0.5*numPoints && disparaty <Dis_limit
            setPoints(tracker, KLT_currPoints);
            continue;    
        end
        Frame_count=Frame_count+1;
        %% cheking for keyframe   
        if (sum(validIdx)<0.5* numPoints && disparaty >Dis_limit-25) || disparaty >Dis_limit %|| mod(j, 45) == 0  
            viewId=viewId+1;
            Frame_count=1;
            KLT_currPoints=cornerPoints(abs(KLT_currPoints));
            DHFP= parfeval(pallOp,@detectHarrisFeaturesPer,1,I,roi,numPoints,KLT_currPoints(validIdx));
            RE=1;
            conFI=99.99;          
            %% Making Matches
            matches1= C_Vec(validIdx);
            matches2=(1:sum(validIdx))';
            matchesKeyPt = cat(2,matches1,matches2);
            validPointindex=find(validIdx);         
            %% read the previous points after BA
            [RefinedXYZ,imagePointsAll,camPoses,Reidx]= fetchOutputs(RPP); 
            vSet = updateView(vSet, camPoses);
            %% store points               
            worldpointData{viewId-1}=RefinedXYZ;% storing 3D points                     
            %% generating camera poses          
            [worldPoints, imagePoints] = helperFind3Dto2DCorrespondences(vSet,...
                KLT_currPoints,RefinedXYZ,imagePointsAll,validPointindex,viewId-1);% in here KLT points are given correctly takes 7ms
            %% p3p Pose Generation
            for kk=1:3              
             [orient, loc,inlierIdx2,status] = estimateWorldCameraPose(imagePoints, worldPoints, ...
                    cameraParams, 'Confidence', conFI, 'MaxReprojectionError', RE,'MaxNumTrials', 10000); %average 43ms
                %if sum(inlierIdx2) / length(imagePoints) > 0.1 && sum(inlierIdx2)>10
                if status==0 && sum(inlierIdx2)>numPoints*0.05         
                    startkeypoint=false;
                    break ;
                end
                startkeypoint=false;
                RE=RE+1;
                conFI=conFI-1.5;
                status=0;
            end 
            %% Getting new points
            currPoints=fetchOutputs(DHFP);    %4ms
            %% Storing to vSet
            vSet = addView(vSet, viewId, 'Points', currPoints, 'Orientation', orient, ...
                'Location', loc);%7ms
            vSet = addConnection(vSet, viewId-1, viewId, 'Matches', matchesKeyPt);         
            %% arranging tempary vSet2
            if viewId>5
                vSet2=deleteView(vSet,1:viewId-5);
            else
                vSet2=vSet;
            end
            
            %% Refine pose + imageindex store
            RPP=parfeval(pallOp,@refinepose,4,vSet2,viewId,cameraParams,imagePoints,worldPoints);                       
            %% Tracking Point correction
            prevPoints= currPoints;
            KTL_prevPoints=currPoints.Location;       
            %% setting the tracker
            setPoints(tracker, currPoints.Location);                      
            %% Display functions          
            BB=cell2mat(worldpointData');
            set(ptSHOW, 'XData', BB(1:20:end,1), 'YData',BB(1:20:end,2), 'ZData', BB(1:20:end,3));           
            helperUpdateCameraPlots(viewId,camEstimated,poses(vSet));
            helperUpdateCameraTrajectories(viewId,trajectoryEstimated,poses(vSet));       
        else
            %pause(0.07);
            if Re_Tracking;
            setPoints(tracker,KLT_currPoints,nexttrcIDs);
            end
            if viewId == 15
                axis('manual'); % this will stop movement of the diaplay     
            end
        end
    end



