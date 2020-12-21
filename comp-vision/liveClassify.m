%% Generate a Webcam Program to Capture Training Data with Image Subtraction
close all; clear all;
load 4actionmodelNet.mat;
cam = webcam(1);
cam.Resolution = '1280x720';

% Rectangle Parameters Trained on 512x400
x = 1; y = 1; width = 512; height = 400;

% Image subtraction threshold
threshold = 20;

% Stop Time
stop_time = 150;

% Solve for Background Average
bkgnd=calibrate(cam,x,y,width,height);
bkgnd = rgb2gray(bkgnd);

% Capture Frames
disp("Live Webcam Feed");
for idx = 1:stop_time
    img = snapshot(cam);
    % Show the webcam feed with the capture area in rectangle
    img2 = insertShape(img,'Rectangle',[x y width height],'LineWidth',2,'Color','blue');
    
    % Classify Reserved Region
    img1 = img(x:x+height-1,y:y+width-1,:);
    grey = rgb2gray(img1);
    diff = grey-bkgnd;
    rediff = imresize(diff>threshold,[200 256]);
    frameGestures(1,idx) = classify(net,rediff);
    
    % Display Classification to Figure
    img2 = insertText(img2,[60 8],char(frameGestures(1,idx)));
    
    imshow(img2);
end
disp("Live Webcam Feed End");
% We will have the string representations
gestureClassified = char(frameGestures);

cleanClassification;
clear cam;