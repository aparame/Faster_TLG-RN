clc
clear 
close all


% Jackal Robot safe footprint = 1m
% Jackal Robot footprint = 0.5m


%% Convert from grid to LTL states %%
clc
clear 
close all
trial = 2;
if trial == 2
    image = imread('simple_with_obstacles.jpg');
    grayimage = rgb2gray(image);

    resize = 0.5;
    grayimage = imresize(grayimage,resize,'box')./255;
    bwimage = grayimage == 0;
    bwimage = imresize(bwimage,0.2);
    grid = occupancyMap(bwimage,2);  % smallest cell size should be 0.75m x 0.75m
    
    occupancy_prob = flip(getOccupancy(grid)*100);
    occupancy_prob(42,19) = 99.9;
    occupancy_prob(26,29) = 99.9;
    occupancy_prob(10,21) = 99.9;
    occ_LTL = occupancy_prob > 10;
    occ_LTL = flip(occ_LTL);
    grid_LTL = occupancyMap(occ_LTL);
    show(grid_LTL)
    % occupancy_prob(1,:) = 0.1;occupancy_prob(:,1) = 0.1;occupancy_prob(end,:) = 0.1;
    
    
    start_XY = [5,5];
    goals_XY = [50,10;50,20];  % 2 goals to begin with for a single agent


    gridWidth = width(occupancy_prob);
    gridHeight = height(occupancy_prob);
    start_A = [start_XY(1),gridHeight-start_XY(2)];
    goals_A = [goals_XY(:,1),gridHeight-goals_XY(:,2)];

else 
    occupancy_map = zeros(4,4);
    occupancy_map(2,2) = 1;
    % occupancy_map(4,4) = 1;
    % occupancy_map(6,6) = 1;
    % occupancy_map(8,8) = 1;
    grid = occupancyMap(occupancy_map);  % smallest cell size should be 0.75m x 0.75m
    occupancy_prob = flip(getOccupancy(grid)*100);
    gridWidth = width(occupancy_prob);
    gridHeight = height(occupancy_prob);
%     occupancy_prob(1,:) = 0.1;occupancy_prob(:,1) = 0.1;occupancy_prob(end,:) = 0.1;

    figure()
    show(grid);
    start_XY = [1,1];
    goals_XY = [4,4];  % 2 goals to begin with for a single agent
    start_A = start_XY;
    goals_A = goals_XY;
end



start = convert2array(start_XY,gridWidth);
goals = convert2array(goals_XY,gridWidth);

all_states = occupancy_prob';
all_states = all_states(:)';
obstacles = find(all_states>90);

%% Compare speed with A*
for i=1:length(goals)
    astar = plannerAStarGrid(grid_LTL); 
    astar_path = plan(astar,flip(start_A),flip(goals_A(i,:)));
    % hold on
    % show(astar)
end

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
pathNuSMV = 'C:\Program Files\NuSMV-2.6.0-win64\bin';

sensedObstacles = obstacles;
% sensedObstacles = [];
[xSObs,ySObs] = cellPath2Grid(sensedObstacles,gridWidth, gridHeight);
% sensedObstacles = cell(3,1);
[xObs,yObs] = cellPath2Grid(obstacles,gridWidth, gridHeight);
[xGoal,yGoal] = cellPath2Grid(goals, gridWidth, gridHeight);
[xStart,yStart] = cellPath2Grid(start', gridWidth, gridHeight);

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

xPath = [];
yPath = [];
cPath = [];
gridChunk = zeros(1,4);
mTrack = [];


expansion = -1;
while isempty(cPath) || length(cPath) < 50
    
    expansion = expansion + 1;
    
    gridChunk = getChunk(start, goals, gridWidth, gridHeight,expansion);
    
    hold on
    rectangle('Position',[gridChunk(1)-.5, gridChunk(3)-.5, ...
        gridChunk(2)-gridChunk(1)+1, gridChunk(4)-gridChunk(3)+1],...
        'LineWidth',6)
    hold off
    
    pause(.1)
    
    makeSMV_v2(fileName, gridWidth, gridHeight, gridChunk, start, sensedObstacles, ltlspec);
    
    [~, output] = system(['cd ' pathNuSMV ' & NuSMV ' filePath '\' fileName]);
    
    [xPath,yPath,cPath] = getPath(output, gridWidth, gridHeight);
    
end

% For collision detection, save current and next cells
mTrack = cPath;



%% Plot initial plan

figure()
hold off
% plot(xGrid,yGrid,'k')
axis equal
axis([0 gridWidth+1 0 gridHeight+1])
title('Planning'),xlabel('x'),ylabel('y')
hold on
plot(xGoal,yGoal,'dm','MarkerSize',12)
plot(xSObs,ySObs,'xr','MarkerSize',5)

plot(xPath(2),yPath(2),'ko',xPath(end),yPath(end),'k^',...
    'MarkerSize',12)
plot(xPath(3:end),yPath(3:end),'k',astar_path(:,2),gridHeight-astar_path(:,1),'r')

hold off





















%%
function X = convert2array(locations, gridW)
X = zeros([1,height(locations)]);
for i = 1:height(locations)
    X(i) = (locations(i,2)-1)*gridW + locations(i,1);
end
end



% S = qtdecomp(grayimage);
% blocks = repmat(uint8(0),size(S));
% 
% for dim = [1000 500 250 125 62 31 16 8 4 2 1]    
%   numblocks = length(find(S==dim));    
%   if (numblocks > 0)        
%     values = repmat(uint8(1),[dim dim numblocks]);
%     values(2:dim,2:dim,:) = 0;
%     blocks = qtsetblk(blocks,S,dim,values);
%   end
% end
% 
% blocks(end,1:end) = 1;
% blocks(1:end,end) = 1;
% figure(2)
% imshow(blocks,[])