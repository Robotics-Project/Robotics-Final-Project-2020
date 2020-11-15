%% Generate a Webcam Program to Capture Training Data with Image Subtraction
%clear all; close all;
close all; clear all;
load 4actionmodelNet.mat;
cam = webcam(1);
cam.Resolution = '1280x720';

% Rectangle Parameters Trained on 512x400
x = 1; y = 1; width = 512; height = 400;
x2 = 767;
% Image subtraction threshold
threshold = 20;

% Solve for Background Average
bkgnd=calibrate(cam,x,y,width,height);
bkgnd = rgb2gray(bkgnd);

% Where to store images to
dir = "C:\Users\zhug4\Desktop\Robotics\testData\";
class = "vid1";

% Capture Frames
disp("Live Webcam Feed");
for idx = 1:500
    img = snapshot(cam);
    % Show the webcam feed with the capture area in rectangle
    img2 = insertShape(img,'Rectangle',[x y width height],'LineWidth',2,'Color','blue');
    img2 = insertShape(img2,'Rectangle',[x2 y width height],'LineWidth',2,'Color','blue');
    %imshow(img2);
    
    img1 = img(x:x+height-1,y:y+width-1,:);
    grey = rgb2gray(img1);
    diff = grey-bkgnd;
    rediff = imresize(diff>threshold,[200 256]);
    frameGestures(1,idx) = classify(net,rediff);
    img2 = insertText(img2,[60 5],char(frameGestures(1,idx)));
    
    imshow(img2);
    %imwrite(img2,dir+class+"\"+int2str(idx)+".png");
end
disp("Live Webcam Feed End");
% We will have the string representations
gesturesClassified = char(frameGestures);

clear cam;