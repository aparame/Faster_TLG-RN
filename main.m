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

[map,map_LTL,model] = select_map(model,'complexMap','custom',0);   % 'simple' and 'custom/built-in'

tic;
[model,LTL_path] = LTL_Planning(model);
model.LTL_path = LTL_path(3:end,:);
model.LTL_path(end+1,:) = model.goals_XY(1,:);
                                                %%  Known Environment with A* %%
% model.astar_path = AStar(model,map);
% model.astar_path = [model.astar_path(:,2),map.GridSize(1,2)-model.astar_path(:,1)];


% % A Star Path Plotting (black)
% plot(model.init_pos(1,1),model.init_pos(1,2),'.','MarkerSize',30);
% arrow([model.astar_path(1:arrow_distance:end-1,1),model.astar_path(1:arrow_distance:end-1,2)],...
%     [model.astar_path(2:arrow_distance:end,1),model.astar_path(2:arrow_distance:end,2)],2); % plot path as arrows



                                            %% Get Robustness and Prediction Horizon %%
rho_req = 2;
[model.min_rob,model.sdm] = get_robustness(model.LTL_path,map);     % STL spec min_d[t] > 2
model.ds = 1;

                                         %% Known Environment with LTL and STL based Path Planning (second step)

% Setting up pre-requisities
model.hor_length = (size(model.LTL_path,1)-1)/model.ds;             
model.u0.min = model.u0.min(:,ones(1,model.hor_length+1)); % minimum control input ---> set in system description code
model.u0.max = model.u0.max(:,ones(1,model.hor_length+1)); % maximum control input

% MILP Problem solver
model = generate_STL_path(model,map);
model.STL_path = model.solutions{1,3};

toc;


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
for ii = 1:model.n_obs
    fill(model.obs(:,1,ii),model.obs(:,2,ii),[0 0 0]);  
end
% LTL Path (Red)
plot(LTL_path(2,1),LTL_path(2,2),'ko',LTL_path(end,1),LTL_path(end,2),'k^',...
    'MarkerSize',6);
plot(LTL_path(size(model.goals,3)+1:end,1),LTL_path(size(model.goals,3)+1:end,2),'b','LineWidth',3);
plot(model.init_pos(1,1),model.init_pos(1,2),'.','MarkerSize',30)
plot(model.STL_path(1,:),model.STL_path(2,:),'r','LineWidth',3)
title('LTL and STL Safe Path Planning')

hold off

                                            %% Get Robustness for STL %%

[model.min_rob,model.sdm] = get_robustness(model.STL_path',map);     % STL spec min_d[t] > 2







































