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


% file locations
fileName = 'ADI_LTL.smv';
filePath = cd;
pathNuSMV = '/home/yue6/NuSMV-2.6.0-Linux/bin';

sensedObstacles = obstacles;
% sensedObstacles = [];
% [xSObs,ySObs] = cellPath2Grid(sensedObstacles,model.gridWidth, model.gridHeight);
% sensedObstacles = cell(3,1);
% [xObs,yObs] = cellPath2Grid(obstacles,model.gridWidth, model.gridHeight);
% [xGoal,yGoal] = cellPath2Grid(goals, model.gridWidth, model.gridHeight);
% [xStart,yStart] = cellPath2Grid(start', model.gridWidth, model.gridHeight);

% %% Plot Enviornmnet
%
% xText = [];
% yText = [];
% for i = 1:gridHeight
%     xText = [xText, 1:gridWidth];
%     yText = [yText, i*ones(1, gridWidth)];
% end
%
% xGrid = [1:2:gridWidth+1, ones(1,gridHeight+2); 1:2:gridWidth+1, (gridWidth+1)*ones(1,gridHeight+2)]-.5;
% yGrid = [ones(1,gridWidth+1), 1:2:gridHeight+1; (gridWidth+1)*ones(1,gridWidth+1), 1:2:gridHeight+1]-.5;
%
% figure(1)
% subplot(1,2,1)
% hold off
% plot(xGrid,yGrid,'k')
% axis equal
% axis([0 gridWidth+1 0 gridHeight+1])
% hold on
% % text(xText+.1,yText+.25,num2str([1:gridWidth*gridHeight]'))
% plot(xGoal, yGoal, 'dm', 'MarkerSize', 12)  %diamond
% plot(xSObs,ySObs,'xr','MarkerSize',15)
% plot(xStart,yStart,'ok','MarkerSize',12)  %circle - Start points
% title('Pathing'),xlabel('x'),ylabel('y')
% hold off
%
% pause(.1)


%% Planning

cPath = [];


expansion = -1;
while isempty(cPath) || length(cPath) < 50

    expansion = expansion + 1;

    gridChunk = getChunk(start, goals, model.gridWidth, model.gridHeight,expansion);

    % hold on
    % rectangle('Position',[gridChunk(1)-.5, gridChunk(3)-.5, ...
    %     gridChunk(2)-gridChunk(1)+1, gridChunk(4)-gridChunk(3)+1],...
    %     'LineWidth',6)
    % hold off
    % 
    % pause(.1)

    makeSMV_v2(fileName, model.gridWidth, model.gridHeight, gridChunk, start, sensedObstacles, ltlspec);

    [~, output] = system(['source ~/.bashrc & cd ' pathNuSMV ' & NuSMV ' filePath '/' fileName]);

    [xPath,yPath,cPath] = getPath(output, model.gridWidth, model.gridHeight);
    
    LTL_path = [xPath',yPath'];

end

% For collision detection, save current and next cells
mTrack = cPath;

function X = convert2array(locations, gridW)
X = zeros([1,height(locations)]);
for i = 1:height(locations)
    X(i) = (locations(i,2)-1)*gridW + locations(i,1);
end
end




end

