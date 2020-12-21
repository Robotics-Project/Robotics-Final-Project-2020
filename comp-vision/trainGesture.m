digitDatasetPath = fullfile('C:','Users','zhug4','Desktop','Robotics','gestures');
imds = imageDatastore(digitDatasetPath, ...
    'IncludeSubfolders',true, ...
    'LabelSource','foldernames');
imds.ReadFcn = @(loc)imresize(imread(loc),[200 256]);

numTrainingFiles = 3600;
[imdsTrain,imdsTest] = splitEachLabel(imds,numTrainingFiles,'randomize');

%[240 640 1]
layers = [ ...
    imageInputLayer([200 256 1])
    convolution2dLayer(3,64)
    reluLayer
    maxPooling2dLayer(2,'Stride',2)
    convolution2dLayer(3,64)
    reluLayer
    maxPooling2dLayer(2,'Stride',2)
    fullyConnectedLayer(4)
    softmaxLayer
    classificationLayer];

options = trainingOptions('sgdm', ...
    'MaxEpochs',5,...
    'MiniBatchSize',64, ...
    'InitialLearnRate',1e-3, ...
    'Verbose',false, ...
    'Plots','training-progress');

net = trainNetwork(imdsTrain,layers,options);

gestureRec1 = net;
% save gestureRec1


YPred = classify(net,imdsTest);
YValidation = imdsTest.Labels;
accuracy = mean(YPred == YValidation);

% label = classify(net,img);
