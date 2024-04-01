function [model,LTL_path] = LTL_Planning(model)

goals = model.goals;
start_XY = model.init_pos;

for ii = 1:size(goals,3)
    goals_poly = polyshape(goals(:,1,ii),goals(:,2,ii));
    [goal_centre_x,goal_centre_y] = centroid(goals_poly);
    goals_XY(ii,1) = round(goal_centre_x);
    goals_XY(ii,2) = round(goal_centre_y);
end

model.goals_XY = goals_XY;
start = convert2array(start_XY,model.gridWidth);
goals = convert2array(goals_XY,model.gridWidth);

all_states = model.occupancy_prob';
all_states = all_states(:)';
obstacles = find(all_states>0.9);

%% Convert to NuSMV file %%
M = 1 ; % no. of agents
ltlspec = cell(M,1);

ltlspec = ['LTLSPEC ! (( F (x.state = ',num2str(goals(1)),')'];  % str()
for i = 2:length(goals)
    ltlspec = [ltlspec, ' & F (x.state = ',num2str(goals(i)),')']; % ltlspec.append('& F (x.state = ',num2str(goals(i)),')')
end
ltlspec = [ltlspec, ' ))'];

%%
% file locations
fileName = 'ADI_LTL.smv';
filePath = cd;
pathNuSMV = '/home/yue6/NuSMV-2.6.0-Linux/bin';

sensedObstacles = obstacles;



%% Planning

cPath = [];


expansion = -1;
while isempty(cPath) || length(cPath) < 5

    expansion = expansion + 1;

    gridChunk = getChunk(start, goals, model.gridWidth, model.gridHeight,expansion);
    makeSMV_v2(fileName, model.gridWidth, model.gridHeight, gridChunk, start, sensedObstacles, ltlspec);

    [~, output] = system(['source ~/.bashrc & cd ' pathNuSMV ' & NuSMV ' filePath '/' fileName]);

    [xPath,yPath,cPath] = getPath(output, model.gridWidth, model.gridHeight);
    
    LTL_path = [xPath',yPath'];

end

% For collision detection, save current and next cells
mTrack = cPath;


end





