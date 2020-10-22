digitDatasetPath = fullfile('C:','Users','zhug4','Desktop','Robotics','archive');
imds = imageDatastore(digitDatasetPath, ...
    'IncludeSubfolders',true, ...
    'LabelSource','foldernames');
imds.ReadFcn = @(loc)imresize(imread(loc),[120 320]);

%{
figure
numImages = 20000;
perm = randperm(numImages,20);
for i = 1:20
    subplot(4,5,i);
    imshow(imds.Files{perm(i)});
    drawnow;
end
%}
    
numTrainingFiles = 1750;
[imdsTrain,imdsTest] = splitEachLabel(imds,numTrainingFiles,'randomize');

%[240 640 1]
layers = [ ...
    imageInputLayer([120 320 1])
    convolution2dLayer(3,64)
    reluLayer
    maxPooling2dLayer(2,'Stride',2)
    convolution2dLayer(3,64)
    reluLayer
    maxPooling2dLayer(2,'Stride',2)
    fullyConnectedLayer(10)
    softmaxLayer
    classificationLayer];

options = trainingOptions('sgdm', ...
    'MaxEpochs',5,...
    'MiniBatchSize',64, ...
    'InitialLearnRate',1e-4, ...
    'Verbose',false, ...
    'Plots','training-progress');

net = trainNetwork(imdsTrain,layers,options);

gestureRec1 = net;
% save gestureRec1


YPred = classify(net,imdsTest);
YValidation = imdsTest.Labels;
accuracy = mean(YPred == YValidation);

% label = classify(net,img);
