%{
%% Load and Open Model
mdl = 'robot.slx';
%open_system(mdl);

%% Create Environment Interface
numObs = 44;
obsInfo = rlNumericSpec([numObs 1]);
obsInfo.Name = 'observations';

numAct = 8;
actInfo = rlNumericSpec([numAct 1],'LowerLimit',-1,'UpperLimit', 1);
actInfo.Name = 'angle';

blk = [mdl, '/RL Agent'];
env = rlSimulinkEnv(mdl,blk,obsInfo,actInfo);

env.ResetFcn = @quadrupedResetFcn;
%}