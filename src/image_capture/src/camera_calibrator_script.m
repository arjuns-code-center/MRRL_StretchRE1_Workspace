% Auto-generated by cameraCalibrator app on 25-Sep-2023
%-------------------------------------------------------


% Define images to process
imageFileNames = {'E:\Research\UMass_MRRL\MRRL_StretchRE1_Workspace\src\image_capture\src\Images\image_0.png',...
    'E:\Research\UMass_MRRL\MRRL_StretchRE1_Workspace\src\image_capture\src\Images\image_1.png',...
    'E:\Research\UMass_MRRL\MRRL_StretchRE1_Workspace\src\image_capture\src\Images\image_2.png',...
    'E:\Research\UMass_MRRL\MRRL_StretchRE1_Workspace\src\image_capture\src\Images\image_3.png',...
    'E:\Research\UMass_MRRL\MRRL_StretchRE1_Workspace\src\image_capture\src\Images\image_4.png',...
    'E:\Research\UMass_MRRL\MRRL_StretchRE1_Workspace\src\image_capture\src\Images\image_5.png',...
    'E:\Research\UMass_MRRL\MRRL_StretchRE1_Workspace\src\image_capture\src\Images\image_6.png',...
    'E:\Research\UMass_MRRL\MRRL_StretchRE1_Workspace\src\image_capture\src\Images\image_7.png',...
    'E:\Research\UMass_MRRL\MRRL_StretchRE1_Workspace\src\image_capture\src\Images\image_9.png',...
    'E:\Research\UMass_MRRL\MRRL_StretchRE1_Workspace\src\image_capture\src\Images\image_10.png',...
    };
% Detect calibration pattern in images
detector = vision.calibration.monocular.CheckerboardDetector();
[imagePoints, imagesUsed] = detectPatternPoints(detector, imageFileNames);
imageFileNames = imageFileNames(imagesUsed);

% Read the first image to obtain image size
originalImage = imread(imageFileNames{1});
[mrows, ncols, ~] = size(originalImage);

% Generate world coordinates for the planar pattern keypoints
squareSize = 35;  % in units of 'millimeters'
worldPoints = generateWorldPoints(detector, 'SquareSize', squareSize);

% Calibrate the camera
[cameraParams, imagesUsed, estimationErrors] = estimateCameraParameters(imagePoints, worldPoints, ...
    'EstimateSkew', false, 'EstimateTangentialDistortion', false, ...
    'NumRadialDistortionCoefficients', 2, 'WorldUnits', 'millimeters', ...
    'InitialIntrinsicMatrix', [], 'InitialRadialDistortion', [], ...
    'ImageSize', [mrows, ncols]);

% View reprojection errors
h1=figure; showReprojectionErrors(cameraParams);

% Visualize pattern locations
h2=figure; showExtrinsics(cameraParams, 'CameraCentric');

% Display parameter estimation errors
displayErrors(estimationErrors, cameraParams);

% For example, you can use the calibration data to remove effects of lens distortion.
undistortedImage = undistortImage(originalImage, cameraParams);

% For use in OpenCV
[cameraMatrix,distortionCoefficients] = cameraIntrinsicsToOpenCV(cameraParams);
save("E:\Research\UMass_MRRL\MRRL_StretchRE1_Workspace\src\image_capture\src\camParams.mat", "cameraMatrix", "distortionCoefficients");

% See additional examples of how to use the calibration data.  At the prompt type:
% showdemo('MeasuringPlanarObjectsExample')
% showdemo('StructureFromMotionExample')
