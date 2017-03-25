clear;
close all;
%% Camera Calibration
imageFileNames = {'/Users/e.kim4/Documents/MATLAB/IMG_0001.png',... '/Users/e.kim4/Documents/MATLAB/IMG_0002.png',... '/Users/e.kim4/Documents/MATLAB/IMG_0003.png',... '/Users/e.kim4/Documents/MATLAB/IMG_0004.png',... '/Users/e.kim4/Documents/MATLAB/IMG_0005.png',... '/Users/e.kim4/Documents/MATLAB/IMG_0006.png',... '/Users/e.kim4/Documents/MATLAB/IMG_0007.png',... '/Users/e.kim4/Documents/MATLAB/IMG_0008.png',... '/Users/e.kim4/Documents/MATLAB/IMG_0009.png',... '/Users/e.kim4/Documents/MATLAB/IMG_0010.png',... '/Users/e.kim4/Documents/MATLAB/IMG_0011.png',... '/Users/e.kim4/Documents/MATLAB/IMG_0012.png',... '/Users/e.kim4/Documents/MATLAB/IMG_0013.png',... '/Users/e.kim4/Documents/MATLAB/IMG_0014.png',... '/Users/e.kim4/Documents/MATLAB/IMG_0015.png',... '/Users/e.kim4/Documents/MATLAB/IMG_0016.png',... '/Users/e.kim4/Documents/MATLAB/IMG_0017.png',... '/Users/e.kim4/Documents/MATLAB/IMG_0018.png',... '/Users/e.kim4/Documents/MATLAB/IMG_0019.png',... '/Users/e.kim4/Documents/MATLAB/IMG_0020.png',...
};
% Detect checkerboards in images
[imagePoints, boardSize, imagesUsed] = detectCheckerboardPoints(imageFileNames); imageFileNames = imageFileNames(imagesUsed);
% Generate world coordinates of the corners of the squares squareSize = 25; % in units of 'mm'
worldPoints = generateCheckerboardPoints(boardSize, squareSize);
% Calibrate the camera
[cameraParams, imagesUsed, estimationErrors] = estimateCameraParameters(imagePoints, worldPoints, ...
'EstimateSkew', false, 'EstimateTangentialDistortion', false, ... 'NumRadialDistortionCoefficients', 2, 'WorldUnits', 'mm', ... 'InitialIntrinsicMatrix', [], 'InitialRadialDistortion', []);
% View reprojection errors
%h1=figure; showReprojectionErrors(cameraParams, 'BarGraph');
% Visualize pattern locations
%h2=figure; showExtrinsics(cameraParams, 'CameraCentric');
% Display parameter estimation errors
 
 
%displayErrors(estimationErrors, cameraParams);
% For example, you can use the calibration data to remove effects of lens distortion.
originalImage = imread(imageFileNames{1});
undistortedImage = undistortImage(originalImage, cameraParams);
extrinsicMatrix = [cameraParams.RotationVectors cameraParams.TranslationVectors];
% Show Intrinsics and Extrinsic
cameraParams.IntrinsicMatrix 
extrinsicMatrix

%% Image capturing for 2 images
images1 = rgb2gray(imresize(imread('IMG_0021.png'),[720,720]));
images2 = rgb2gray(imresize(imread('IMG_0022.png'),[720,720]));

%% Finding Interest Points
close all;
% matched points between images1 and images2
interestPoints = detectMinEigenFeatures(images1); interestPoints = interestPoints.selectStrongest(200); pointTracker = vision.PointTracker();
 
initialize(pointTracker,interestPoints.Location, images1); [stepOutcome1, stepOutcome2] = step(pointTracker,images2); stepOutcome = [stepOutcome1 stepOutcome2];
stepOutcome = round(stepOutcome);
% finding matched points by checking validity on images1 and on % images2
[numRow,numColumn] = size(stepOutcome);
m = 1;
for n = 1:numRow
    if stepOutcome(n,3) == 1
    matchedLocOriginal(m,:) = [interestPoints.Location(n,1) interestPoints.Location(n,2)];
    matchedLocCompare(m,:) = [stepOutcome(n,1) stepOutcome(n,2)];
    m = m+1;
    end
end
figure(1);
showMatchedFeatures(images1, images2, matchedLocOriginal, matchedLocCompare)
title('Matched point between image1 to image2')

%% Estimation of Camera Matrices
[F1, inliners] = estimateFundamentalMatrix(matchedLocOriginal,matchedLocCompare);
 
m = 1;
for n = 1:150
    if inliners(n) == 1
    inlinersOriginal(m,:) = [matchedLocOriginal(n,1) matchedLocOriginal(n,2)]; inlinersCompare(m,:) = [matchedLocCompare(n,1) matchedLocCompare(n,2)];
    m = m+1;
    end
end

%% Reconstruction and Triangulation
[relativeOrientation, relativeTranslation] = cameraPose(F1,cameraParams,inlinersOriginal,inlinersCompare);
for n = 1:3
    intrinsicMatrix(n,:) = [cameraParams.IntrinsicMatrix(n,:) 0];
end
%intrinsicMatrix(4,:) = [0 0 0 1];
cameramatrix1 = cameraMatrix(cameraParams,eye(3),[0 0 0]);
cameramatrix2 =cameraMatrix(cameraParams, relativeOrientation, relativeTranslation); %cameramatrix1 = intrinsicMatrix * cameraMatrix(cameraParams,eye(3),[0 0 0]); %cameramatrix2 =intrinsicMatrix * cameraMatrix(cameraParams, relativeOrientation, relativeTranslation);
%figure(1)
%showMatchedFeatures(images1, images2, matchedLocOriginal, matchedLocCompare,'montage','PlotOptions',{'ro','go','y--'});

%% Triangulation
identityMatrix = [1 0 0 0; 0 1 0 0;
0 0 1 0];
 
reconstructed = triangulate(matchedLocOriginal, matchedLocCompare, cameramatrix1, cameramatrix2);
%figure(2)
%showMatchedFeatures(images1, images2, matchedLocOriginal(inliners,:),matchedLocCompare(inliners,:),'montage','PlotOptions',{ 'ro','go','y--'});
%figure(3)
%showMatchedFeatures(images1, images2, matchedLocOriginal(inliners1,:),matchedLocCompare(inliners1,:),'montage','PlotOptions' ,{'ro','go','y--'});

%% 3D Reconstruction
%[bApoints, camPoses, reprojectionErrors] = bundleAdjustment(reconstructed,tracks,camPoses, cameraParams)
% Create the point cloud
ptCloud = pointCloud(reconstructed);
figure
cameraSize = 0.2;
plotCamera('Size', cameraSize, 'color', 'r', 'Label', '1', 'Opacity', 0); hold on
grid on
plotCamera('Location', relativeTranslation, 'Orientation', relativeOrientation, 'Size', cameraSize,'Color', 'b', 'Label', '2', 'Opacity', 0);
% Visualize the point cloud
pcshow(ptCloud, 'VerticalAxis', 'y', 'VerticalAxisDir', 'down', 'MarkerSize', 200);
% Rotate and zoom the plot
camorbit(5, -35); camzoom(1.0);
