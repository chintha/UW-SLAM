function img=enhancefun(img);
img = im2double(img);

sigma=100;
Tol=[0.02 0.98];
%img= rgb2gray(img);
img = imguidedfilter(img,'NeighborhoodSize',[4 4],'DegreeOfSmoothing',0.0000001);   
[img,T,L]=imreducehazeedit(img,'method','simpledcp','ContrastEnhancement','global');
img=imflatfield(img,sigma);
%T=uint8(T);

D = -log(1-T+eps);
D = mat2gray(D);
%img1 = adapthisteq(img,'NumTiles',[8 8],'ClipLimit',0.08,'Distribution','rayleigh');
img1 = imadjust(img,stretchlim(img,Tol),[]); %this give better result than adaptive contrast stretching
img2=img1-img;

img3=(0.5*img2).*(1-D);
img=img1+0.1*img3;
img = imsharpen(img,'Radius',1,'Amount',0.8,'Threshold',0.5);

%img = rgb2ycbcr(img);
%img = imadjust(img,stretchlim(img,Tol),[]);

%img=ycbcr2rgb(img);


%figure
%imshowpair(img,img2,'montage')

end
