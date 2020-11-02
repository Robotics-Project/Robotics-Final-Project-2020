%% Generate a Webcam Program to Capture Training Data with Image Subtraction
clear all; close all;
cam = webcam(1);
cam.Resolution = '1280x720';

% Rectangle Parameters
x = 50; y = 1; width = 512; height = 400;

% Image subtraction threshold
threshold = 20;

% Solve for Background Average
bkgnd=calibrate(cam,x,y,width,height);
bkgnd = rgb2gray(bkgnd);

% Where to store images to
dir = "C:\Users\zhug4\Desktop\Robotics\gestures\";
class = "02_bunny";

disp("Live Webcam Feed");
for idx = 1:2000
    img = snapshot(cam);
    %img2 = insertShape(img,'Rectangle',[x y width height],'LineWidth',2,'Color','blue');
    img1 = img(x:x+height-1,y:y+width-1,:);
    grey = rgb2gray(img1);
    diff = grey-bkgnd;
    %imshow(img2);
    imshow(diff>threshold);
    imwrite(diff>threshold,dir+class+"\"+int2str(idx)+".png");
end
disp("Live Webcam Feed End");