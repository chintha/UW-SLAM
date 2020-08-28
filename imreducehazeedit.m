function [finalDehazedOutput, T, atmLight] = imreducehazeedit(im, varargin)
%IMREDUCEHAZE  Reduce atmospheric haze.
%   [D, T, L] = IMREDUCEHAZE(X) reduces atmospheric haze in X, which is an
%   RGB or grayscale image. D is the dehazed image. T contains an estimate
%   of the haze thickness at each pixel. L is the estimated atmospheric
%   light, which represents the value of the brightest non-specular haze.
%
%   [___] = IMREDUCEHAZE(X, AMOUNT) alters the AMOUNT of haze removed.
%   AMOUNT is a scalar value in the range [0, 1]. When AMOUNT is 1 (the
%   default), the maximum amount of haze is reduced. When AMOUNT is 0, the
%   input image is unchanged. The default value is 1.
%
%   [___] = IMREDUCEHAZE(___, PARAM, VAL) changes the behavior of the
%   dehazing algorithm using the following name-value pairs.
%
%     'Method'               The technique used to reduce haze: 'simpledcp'
%                            (default) or 'approxdcp'. The 'simpledcp'
%                            method employs a per-pixel dark channel prior
%                            to estimate haze and quadtree decomposition to
%                            compute the atmospheric light, while 'approxdcp'
%                            uses both per-pixel and spatial blocks when
%                            computing the dark channel prior and does not
%                            use quadtree decomposition.
%
%     'AtmosphericLight'     A 1-by-3 vector (for RGB images) or a scalar
%                            (for grayscale) containing the maximum value
%                            to be treated as haze. When not specified, this
%                            value is computed based on the method. For
%                            approxdcp, the brightest 0.1% pixels of the
%                            dark channel are considered to estimate the
%                            value. For simpledcp, it is computed with
%                            quadtree decompostition. 
%
%     'ContrastEnhancement'  A post-processing technique to improve image
%                            contrast: 'global' (default), 'boost', or 'none'.
%
%     'BoostAmount'          A scalar in the range [0, 1] giving the
%                            percentage of per-pixel gain to apply as post-
%                            processing. The default is 0.1. This parameter
%                            is only allowed if 'ContrastEnhancement' is
%                            'boost'.
%
%   Class Support
%   -------------
%   X must be a real, non-sparse, M-by-N-by-3 RGB or M-by-N grayscale image.
%
%   Examples
%   --------
%   % 1 - Reduce haze using the default parameters.
%   A = imread('foggysf1.jpg');
%   B = imreducehaze(A);
%   figure, imshowpair(A, B, 'montage')
%
%   % 2 - Reduce haze 90% using approxdcp and global contrast stretching.
%   A = imread('foggysf2.jpg');
%   B = imreducehaze(A, 0.9, 'method', 'approxdcp');
%   figure, imshowpair(A, B, 'montage')
%
%   % 3 - Estimate the haze thickness and image depth.
%   A = imread('foggyroad.jpg');
%   [~, T] = imreducehaze(A);
%   figure, imshowpair(A, T, 'montage')
%
%   % The haze thickness provides a rough approximation of the depth of the
%   % scene, defined up to an unknown multiplication factor. Add eps to
%   % avoid log(0).
%   D = -log(1-T+eps);
%
%   % For display purposes, scale the depth so that it is in [0,1].
%   D = rescale(D);
%
%   % Display the original image next to the estimated depth in false color.
%   figure
%   subplot(1,2,1)
%   imshow(A), title('Hazy image')
%   subplot(1,2,2)
%   imshow(D), title('Depth estimate')
%   colormap(gca, hot(256))
%
%   References
%   ---------
%   [1] He, Kaiming. "Single Image Haze Removal Using Dark Channel Prior."
%   Thesis, The Chinese University of Hong Kong, 2011.
%   [2] Dubok et al. "Single Image Dehazing with Image Entropy and
%   Information Fidelity." ICIP, 2014.

%   Copyright 2017-2018 The MathWorks, Inc.

[A, amount, method, atmLight, ...
    contrastMethod, boostAmount] = parseInputs(im, varargin{:});
isRGB = (ndims(A) == 3) && (size(A,3) == 3);
originalClass = class(A);

if isfloat(A)
    % The algorithm assumes the input image is in [0,1]
    A = min(1, max(0, A));
else
    % convert to single if image is not float type
    A = im2single(A);
end
amount = cast(amount,'like', A);

if (amount == 0)
    finalDehazedOutput = A;
    T = [];
    atmLight = [];
else
    % Choose method to dehaze the image
    switch method
        case 'simpledcp'
            [deHazed, T, atmLight] = deHazeSimpleDCP(A, amount, atmLight, isRGB);
        case 'approxdcp'
            [deHazed, T, atmLight] = deHazeApproxDCP(A, amount, atmLight, isRGB);
        otherwise
            assert(false);
    end
    
    % Select method of contrast enhancement (Post-processing)
    switch contrastMethod
        case 'global'
            finalDehazedOutput = globalStretching(deHazed);
        case 'boost'
            finalDehazedOutput = boosting(deHazed, amount, boostAmount, 1-T);
        case 'none'
            finalDehazedOutput = deHazed;
        otherwise
            assert(false);
    end
end

% Convert dehazed image and transmission map back to input image class
finalDehazedOutput = convertToOriginalClass(finalDehazedOutput, originalClass);
T = double(T);

end


function [B, T,atmLight] = deHazeSimpleDCP(A, amount, atmLight, isRGB)
% SimpleDCP
% This function computes dark-channel prior, and refines it using guided
% filter, only across channel elements are considered for dark channel
% estimation

% 1. Estimate atmospheric light
if isempty(atmLight)
    atmLight = computeatmLightUsingQuadTree(A);
    if isRGB
        atmLight = reshape(atmLight, [1 1 3]);
    end
else
    if isRGB
        atmLight = reshape(atmLight, [1 1 3]);
    end
end

% 2. Estimate transmission t(x)
normI = min(A, [] , 3);
transmissionMap = 1 - normI;

% 3. Use guided filtering to refine the transmission map
epsilon = 0.01; % default value
transmissionMap = images.internal.algimguidedfilter(transmissionMap,...
    transmissionMap, [5 5], epsilon);
transmissionMap = min(1, max(0, transmissionMap));
omega = 0.9;

% Thickness of haze in input image is second output of imreducehaze.
% Thickness Map does not depends on amount value.
T = 1 - transmissionMap;

% Omega value is set to 0.9, to leave some of haze in restored image for
% natural appearance of dehazed scene
transmissionMap = 1 - omega * (1 - transmissionMap);

% This lower bound preserves a small amount of haze in dense haze regions
t0 = cast(0.1,'like',A);

% Recover scene radiance
radianceMap = atmLight + (A - atmLight) ./ max(transmissionMap, t0);
radianceMap = min(1, max(0, radianceMap));

% New transmission map based on amount of haze to be removed
newTransmissionMap = min(1, transmissionMap + amount);

% Dehazed output image based on Amount, if Amount == 1,
% then B = radianceMap
B = radianceMap .* newTransmissionMap + ...
    atmLight .* (1-newTransmissionMap);

% Reshape atmLight to 1 x 3 vector if input image is RGB
if(isRGB)
    atmLight = double(reshape(atmLight, [1 3]));
else
    atmLight = double(atmLight);
end

end

function [B, T,atmLight] = deHazeApproxDCP(A, amount, atmLight,isRGB)
% DCP
% This function computes dark-channel prior, and refines it using guided
% filter, spatial and across channel elements are considered

% 1. Calculate dark channel image prior
patchSize = ceil(min(size(A,1), size(A,2)) / 400 * 15);
minFiltStrel = strel('square', patchSize);

darkChannel = min(A,[],3);
darkChannel = imerode(darkChannel, minFiltStrel);

% 2. Estimate atmospheric light
if isempty(atmLight)
    if isRGB
        I = rgb2gray(A);
    else
        I = A;
    end
    atmLight = estimateAtmosphericLight(A, I, darkChannel);
else
    if isRGB
        atmLight = reshape(atmLight, [1 1 3]);
    end
end

% 3. Estimate transmission t(x)
normI = A ./ atmLight;
normI = min(normI, [] , 3);
transmissionMap = 1 - imopen(normI, minFiltStrel);

% 4. Use guided filtering to refine the transmission map
% Neighborhood size and degree of smoothing chosen
% empirically to approximate soft matting as best as possible.
epsilon = 1e-4;
filterRadius = ceil(min(size(A,1), size(A,2)) / 50);
nhoodSize = 2 * filterRadius + 1;
% Make sure that subsampleFactor is not too large
subsampleFactor = 4;
subsampleFactor = min(subsampleFactor, filterRadius);
transmissionMap = images.internal.algimguidedfilter( ...
    transmissionMap, A, [nhoodSize nhoodSize], epsilon, subsampleFactor);
transmissionMap = min(1, max(0, transmissionMap));
omega = 0.95;

% Thickness of haze in input image is second output of
% imreducehaze.Thickness Map does not depends on amount value.
T = 1 - transmissionMap;

% Omega value is set to 0.9, to leave some of haze in restored image for
% natural appearance of dehazed scene
transmissionMap = 1 - omega * (1 - transmissionMap);

% This lower bound preserves a small amount of haze in dense haze regions
t0 = cast(0.1,'like',A);

% Recover scene radiance
radianceMap = atmLight + (A - atmLight) ./ max(transmissionMap, t0);
radianceMap = min(1, max(0, radianceMap));

% New transmission map based on amount of haze to be removed
newTransmissionMap = min(1, transmissionMap + amount);

% Dehazed output image based on Amount, if Amount == 1,
% then B = radianceMap
B = radianceMap .* newTransmissionMap + ...
    atmLight .* (1-newTransmissionMap);

% Reshape atmLight to 1 x 3 vector if input image is RGB

if(isRGB)
    atmLight = double(reshape(atmLight, [1 3]));
else
    atmLight = double(atmLight);
end

end


function atmosphericLight = estimateAtmosphericLight(A, I, darkChannel)
% Atmospheric light estimation using 0.1% brightest pixels in darkchannel

% First, find the 0.1% brightest pixels in the dark channel.
% This ensures that we are selecting bright pixels in hazy regions.
p = 0.001; % 0.1 percent
[histDC, binCent] = imhist(darkChannel);
binWidth = mean(diff(binCent));
normCumulHist = cumsum(histDC)/(size(A,1)*size(A,2));
binIdx = find(normCumulHist >= 1-p);
darkChannelCutoff = binCent(binIdx(1)) - binWidth/2;

% Second, find the pixel with highest intensity in the
% region made of the 0.1% brightest dark channel pixels.
mask = darkChannel >= darkChannelCutoff;
grayVals = I(mask);
[y, x] = find(mask);
[~, maxIdx] = max(grayVals);

atmosphericLight = A(y(maxIdx(1)), x(maxIdx(1)), :);
atmosphericLight(atmosphericLight == 0) = eps(class(A));
atmosphericLight=0.8;
end


function atmLight = computeatmLightUsingQuadTree(A)
% Quad-tree decomposition of dark channel is used for estimation
% of atmospheric light as given in reference paper [2]

[dm,dn,~] = size(A);
Q = [1, 1, dm, dn];

if (dm>=64 && dn>=64)
    % default values
    numLevels = 5; % Decomposition levels
    % Window size for finding spatial minimum value for dark channel
    winSize = ceil(min(size(A,1),size(A,2)) / 400 * 15);
    minFiltStrel = strel('square', winSize);
    darkChannel = min(A, [], 3);
    darkChannel = imerode(darkChannel, minFiltStrel);
    
    for ii=1:numLevels
        % Quadrants indices matrix
        quadrantIndex = ([Q(1), Q(2), (Q(1)+Q(3))/2, (Q(2)+Q(4))/2;
            Q(1),((Q(2)+Q(4))/2)+1, (Q(1)+Q(3))/2, Q(4);
            ((Q(1)+Q(3))/2)+1, Q(2), Q(3), ((Q(2)+Q(4))/2);
            ((Q(3)+Q(1))/2)+1, ((Q(4)+Q(2))/2)+1, Q(3), Q(4)]);
        quadrantIndex = round(quadrantIndex);
        
        % Decomposition of dark channel into four quadrants
        firstQuadrant = darkChannel(quadrantIndex(1,1):quadrantIndex(1,3),...
            quadrantIndex(1,2):quadrantIndex(1,4));
        secondQuadrant = darkChannel(quadrantIndex(2,1):quadrantIndex(2,3),...
            quadrantIndex(2,2):quadrantIndex(2,4));
        thirdQuadrant = darkChannel(quadrantIndex(3,1):quadrantIndex(3,3),...
            quadrantIndex(3,2):quadrantIndex(3,4));
        fourthQuadrant = darkChannel(quadrantIndex(4,1):quadrantIndex(4,3),...
            quadrantIndex(4,2):quadrantIndex(4,4));
        
        % Computation of mean for each quadrant
        mu(1) = mean(firstQuadrant(:));
        mu(2) = mean(secondQuadrant(:));
        mu(3) = mean(thirdQuadrant(:));
        mu(4) = mean(fourthQuadrant(:));
        
        % Selecting maximum average intensity quadrant
        [~, ind] = max(mu);
        Q = quadrantIndex(ind, :);
    end
    
    % Selecting bright image pixels based on final decomposed quadrant
    img = A(Q(1):Q(3), Q(2):Q(4), :);
    [mm, nn, pp] = size(img);
    brightIm = ones(mm, nn, pp);
    
    % Minimum Equilidean distance based bright pixel estimation (= atmLight)
    equiDist = sqrt((abs(brightIm - img)).^2);
    equiDistImage = sum(equiDist, 3);
    equiDistVector = equiDistImage(:);
    imageVector = reshape(img, mm*nn, pp);
    [~, index] = min(equiDistVector);
    atmLight = imageVector(index, :);
    
else
    % Selecting bright image pixels based on final decomposed quadrant
    img = A(Q(1):Q(3), Q(2):Q(4), :);
    [mm, nn, pp] = size(img);
    brightIm = ones(mm, nn, pp);
    
    % Minimum Equilidean distance based bright pixel estimation (= atmLight)
    equiDist = sqrt((abs(brightIm-img)).^2);
    equiDistImage = sum(equiDist, 3);
    equiDistVector = equiDistImage(:);
    imageVector = reshape(img, mm*nn, pp);
    [~, index] = min(equiDistVector);
    atmLight = imageVector(index, :);
    
end
atmLight=0.6;
end


function enhanced = globalStretching(A)
% Global Stretching
chkCast = class(A);

% Gamma correction
gamma = 0.75;
A = A.^gamma;

% Normalization to contrast stretch to [0,1]
A = mat2gray(A);

% Find limits to stretch the image
clipLimit = stretchlim(A,[0.001, 0.999]);

% Adjust the cliplimits
alpha = 0.8;
clipLimit = clipLimit + alpha*(max(clipLimit, mean(clipLimit, 2)) - clipLimit);

% Adjust the image intensity values to new cliplimits
enhanced = imadjust(A, clipLimit);
enhanced = cast(enhanced, chkCast);

end


function B = boosting(img, amount, boostAmount, transmissionMap)
% Boost as contrast enhancement technique
boostAmount = boostAmount * (1 - transmissionMap);
B = img .* (1 + (amount * boostAmount));
end


function [im, amount, method, atmLight, contrastMethod, ...
    boostAmount] = parseInputs(im, varargin)
% parsing inputs other than default
narginchk(1, 10);

isRGB = (ndims(im) == 3) && (size(im,3) == 3);
% persistent parser;
parser = inputParser;
parser.FunctionName = mfilename;
parser.CaseSensitive = false;
parser.PartialMatching = true;
parser = inputParser();
validateattributes(im,...
    {'single', 'double', 'uint8', 'uint16'},...
    {'real', 'nonsparse', 'nonempty'}, ...
    mfilename, 'im', 1);
parser.addOptional('amount', 1, @checkAmount);
parser.addParameter('method', 'simpledcp', @checkMethodString);
if isRGB ~= 1
    parser.addParameter('atmosphericLight', [], @checkAtmValueGray);
else
    parser.addParameter('atmosphericLight', [], @checkAtmValue);
end
parser.addParameter('contrastEnhancement', 'global', @checkContrastEnhancementString);
parser.addParameter('boostAmount', [], @checkboostamount);


parser.parse(varargin{:});
amount = parser.Results.amount;
method = validatestring(parser.Results.method, {'simpledcp', 'approxdcp'}, ...
    mfilename, 'method');
atmLight = parser.Results.atmosphericLight;
contrastMethod = validatestring(lower(parser.Results.contrastEnhancement),...
    {'global', 'boost', 'none'},...
    mfilename, 'contrastEnhancement');
boostAmount= parser.Results.boostAmount;

% im must be MxN grayscale or MxNx3 RGB

if ~(ismatrix(im) || isRGB)
    error(message('images:validate:invalidImageFormat', 'im'));
end

switch contrastMethod
    case 'boost'
        if isempty(boostAmount)
            boostAmount = 0.1;
        end
    otherwise
        if ~isempty(boostAmount)
            error(message('images:imreducehaze:boostamountShouldNotBeSpecified'))
        end
end

end

% Check and validate input arguments

function amt = checkAmount(amount)
validateattributes(amount, ...
    {'numeric'}, ...
    {'scalar', 'real', 'nonnegative', '<=', 1, 'nonsparse', 'nonempty'}, ...
    mfilename, 'amount');
amt = true;
end

function tf = checkMethodString(methodString)
validateattributes(methodString, ...
    {'char', 'string'},...
    {'scalartext'},...
    mfilename, 'method');
tf = true;
end

function atm = checkAtmValue(atmLight)
if ~isempty(atmLight)
    validateattributes(atmLight, {'double'}, {'real', 'vector', ...
        'finite', 'nonnegative', '<=', 1},...
        mfilename, 'atmLight');
    
    if (any(size(atmLight) ~= [1,3]))
        error(message('images:imreducehaze:invalidAtmLightVector'))
    end
    
    atm = true;
else
    atm = true;
end
end

function atm = checkAtmValueGray(atmLight)
if ~isempty(atmLight)
    validateattributes(atmLight, {'double'}, {'real', 'scalar', ...
        'finite', 'nonnegative', '<=', 1},...
        mfilename, 'atmLight');
    atm = true;
else
    atm = true;
end
end

function tf = checkContrastEnhancementString(methodString)
validateattributes(methodString, ...
    {'char', 'string'},...
    {'scalartext'},...
    mfilename, 'contrastEnhancement');
tf = true;
end

function checkboostamount = checkboostamount(value)
classes = {'double'};
attributes = {'scalar', 'real',...
    'nonnan', 'positive', 'finite', '<=', 1, 'nonsparse'};
funcName = 'imreducehaze';
validateattributes(value, classes, attributes, funcName);
checkboostamount = true;
end

function B = convertToOriginalClass(B, OriginalClass)

if strcmp(OriginalClass,'uint8')
    B = im2uint8(B);
elseif strcmp(OriginalClass,'uint16')
    B = im2uint16(B);
elseif strcmp(OriginalClass,'single')
    B = im2single(B);
    B = min(1, max(0, B));
else
    %  double
    B = min(1, max(0, B));
end
end
