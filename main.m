%% 3 types of examples to run A* + STL
% The time stamps from the STL spec to the staliro_distance.m help determine the order of waypoints
% to be visited in the original STL spec.
% One way to avoid the time stamps is by introducing the next operator


clc
clear
close all

%% Initialize Model Parameters %%

model = robot_system_description();
model.robot_r = 1/2; % 0.5m robot radius;
%% Known Environment with LTL only %%
% select_map{map_string, custom/inbuilt example, unknown obstacle trigger(0/1)}
% astar_path is a function to calculate the Astar path based on the known
% map, initial positions and the goal
mapName = 'simpleMap';
[map,map_LTL,model] = select_map(model,mapName,'custom',0);   % 'simple' and 'custom/built-in'

tic;
[model,LTL_path] = LTL_Planning(model);
model.LTL_path = LTL_path(3:end,:);
model.LTL_path(end+1,:) = model.goals_XY(1,:);
fprintf('LTL elapsed time is: %.2f seconds. \n',toc')
%%  Known Environment with A* %%
tic;
model.astar_path = AStar(model,map_LTL,mapName);
fprintf('AStar elapsed time is: %.2f seconds. \n',toc')
%%  Known Environment with RRT* %%
tic;
model.rrtstar_path = RRT_star(model,map_LTL,mapName);
fprintf('RRT Star elapsed time is: %.2f seconds. \n',toc')
%% Get Robustness and Prediction Horizon %%
rho_req = 2;
[model.min_rob,model.sdm] = get_robustness(model.LTL_path,map);     % STL spec min_d[t] > 2
model.ds = 1;

%% Known Environment with LTL and STL based Path Planning (second step)

% Setting up pre-requisities
model.hor_length = (size(model.LTL_path,1)-1)/model.ds;
model.u0.min = model.u0.min(:,ones(1,model.hor_length+1)); % minimum control input ---> set in system description code
model.u0.max = model.u0.max(:,ones(1,model.hor_length+1)); % maximum control input

tic;
% MILP Problem solver
model = generate_STL_path(model,map);
model.STL_path = model.solutions{1,3};

fprintf('MILP solver elapsed time is: %.2f seconds. \n',toc')


%% Running the MPC simulation %%
close all
% model.movieName = input('Type in the name of the gif file');
% model.movieName = 2;
model = MPC_plotting(model,map);


%% Plot LTL & STL Path %%
arrow_distance = 8;
figure(2)
show(map)
hold on
for ii = 1:size(model.goals,3)
    fill(model.goals(:,1,ii),model.goals(:,2,ii),[0 0.5 0]);
end

plot(model.init_pos(1,1),model.init_pos(1,2),'.','MarkerSize',30)

% LTL Path (Red)
p1 = plot(model.astar_path(:,2),map.GridSize(1,2)-model.astar_path(:,1),'--k','LineWidth',1.5, 'DisplayName','A*');
p2 = plot(model.rrtstar_path(:,1),model.rrtstar_path(:,2),'--m','LineWidth',1.5,'DisplayName','RRT*');

p3 = plot(LTL_path(size(model.goals,3)+1:end,1),LTL_path(size(model.goals,3)+1:end,2),'b','LineWidth',1.5,'DisplayName','LTL');
p4 = plot(model.STL_path(1,:),model.STL_path(2,:),'r','LineWidth',1.5,'DisplayName','LTL + MILP');
legend([p1,p2,p3,p4])
title('Path Planners comparison (Simple Map)')

hold off

%% Get Robustness for STL %%

[model.min_rob,model.sdm] = get_robustness(model.STL_path',map);     % STL spec min_d[t] > 2




















