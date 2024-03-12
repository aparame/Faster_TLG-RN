function [map,map_LTL,model] = select_map(model,map_selector,goal_selector,unknown_obs)
%% Function to generate the map conditions and goals for each specific map
% Inputs:
%   input = Map option to be selected (str)
%   dynamic = Binary selection to enable unknown obstacles

% Outputs:
%    map = Binary occupancy map based on the input option
%   goals = Goal locations from the map


if strcmp('simpleMap',map_selector)
    image = imread('maps/simpleMap.jpg');
    grayimage = rgb2gray(image);

    resize = 0.5;
    grayimage = imresize(grayimage,resize,'box')./255;
    bwimage = grayimage == 0;
    bwimage = imresize(bwimage,0.2);
    grid = occupancyMap(bwimage,2);  % smallest cell size should be 0.75m x 0.75m

    occupancy_prob = flip(getOccupancy(grid)*100);
    occupancy_prob(1,2) = 99.9;
    occupancy_prob(2,2) = 99.9;
    occupancy_prob(28,2) = 99.9;
    occupancy_prob(27,8) = 99.9;

    model.gridWidth = width(occupancy_prob);
    model.gridHeight = height(occupancy_prob);
    model.occupancy_prob = occupancy_prob;
    occ_LTL = occupancy_prob > 10;
    model.occ_LTL = flip(occ_LTL);
    map = occupancyMap(model.occ_LTL);
    map_LTL = copy(map);
    inflate(map_LTL,1.5)
    model.occupancy_prob = flip(occupancyMatrix(map_LTL));
    show(map_LTL)
    hold on

elseif strcmp('complexMap',map_selector)
    load exampleMaps.mat
    grid = occupancyMap(complexMap);  % smallest cell size should be 0.75m x 0.75m
    occupancy_prob = flip(getOccupancy(grid)*100);
    
    model.occupancy_prob = occupancy_prob > 10;
    model.occupancy_prob = imresize(model.occupancy_prob,2);
    model.gridWidth = width(model.occupancy_prob);
    model.gridHeight = height(model.occupancy_prob);
    map = occupancyMap(model.occupancy_prob);
    map_LTL = copy(map);
    inflate(map_LTL,2)
    model.occupancy_prob = flip(occupancyMatrix(map_LTL));
    show(map_LTL)
    hold on
end


if strcmp('custom',goal_selector)

    [init_x,init_y] = ginputc(1,'ShowPoints',true,'PointColor',  [0.5 0 0]);
    init_pos = round([init_x,init_y]);
    num_goals = input("\nEnter the number of goals");

    for ii = 1:num_goals

        [x_goal,y_goal] = ginputc( 'PointColor',  [0 0.5 0], ...
            'Color',  [0 0.5 0], 'LineWidth', 2, 'ShowPoints', true, ...
            'ConnectPoints', true);
        goals(:,:,ii) = [x_goal,y_goal];
        fill(goals(:,1,ii),goals(:,2,ii),[0 0.5 0]);
    end

elseif strcmp('built-in',goal_selector)
    for ii = 1:2
        goals(:,:,1) = [44,6;
            44,10;
            48,10;
            48,6];
        goals(:,:,2) = [44,18;
            44,22;
            48,22;
            48,18];
        fill(goals(:,1,ii),goals(:,2,ii),[0 0.5 0]);

    end
    init_pos = [7,5];
    plot(init_pos(1,1),init_pos(1,2),'.','MarkerSize',30);
end

if unknown_obs == 1
    n_obstacles = input("\nEnter the number of obstacles unknown to the robot");

    for jj = 1:n_obstacles

        [x_obs,y_obs] = ginputc( 'PointColor',  [0 0.5 0], ...
            'Color',  [0 0.5 0], 'LineWidth', 2, 'ShowPoints', true, ...
            'ConnectPoints', true);
        obstacles(:,:,jj) = round([x_obs,y_obs]);
        fill(obstacles(:,1,jj),obstacles(:,2,jj),[0 0 0]);

    end

else
    n_obstacles = 0;
    obstacles = zeros(2);
end
model.obstacles = obstacles;
model.n_obs = n_obstacles;
model.goals = goals;
model.init_pos = init_pos;
model.x0 = [init_pos';0;0];
end


