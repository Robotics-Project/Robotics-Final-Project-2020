%% Generate a Webcam Program to Capture Training Data with Image Subtraction
%clear all; close all;
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
dir = "C:\Users\zhug4\Desktop\Robotics\gestures\gz\";
class = "buff";

disp("Live Webcam Feed");
for idx = 1:2000
    img = snapshot(cam);
    % Show the webcam feed with the capture area in rectangle
    %img2 = insertShape(img,'Rectangle',[x y width height],'LineWidth',2,'Color','blue');
    %imshow(img2);
    
    img1 = img(x:x+height-1,y:y+width-1,:);
    grey = rgb2gray(img1);
    diff = grey-bkgnd;
    rediff = imresize(diff>threshold,[200 256]);
    disp(classify(net,rediff));
    %disp(idx);
    imshow(diff>threshold);
    %imwrite(diff>threshold,dir+class+"\"+int2str(idx)+".png");
end
disp("Live Webcam Feed End");