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
mapName = 'complexMap';
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
if strcmp('simpleMap',mapName)
    model.astar_path = [model.astar_path(:,2),map.GridSize(1,2)-model.astar_path(:,1)];
else
   model.astar_path = [model.astar_path(:,2),map.GridSize(1,1)-model.astar_path(:,1)];
end
%%  Known Environment with RRT* %%
tic;
model.rrtstar_path = RRT_star(model,map_LTL,mapName);
fprintf('RRT Star elapsed time is: %.2f seconds. \n',toc')
model.ds = 1;

%% Known Environment with LTL and STL based Path Planning (second step)

% Setting up pre-requisities
model.hor_length = (size(model.LTL_path,1)-1)/model.ds;
model.u0.min = model.u0.min(:,ones(1,model.hor_length+1)); % minimum control input ---> set in system description code
model.u0.max = model.u0.max(:,ones(1,model.hor_length+1)); % maximum control input

tic;
% MILP Problem solver
model = generate_STL_path(model,map,model.LTL_path);
model.STL_path = model.solutions{1,3};

fprintf('MILP +LTL solver elapsed time is: %.2f seconds. \n',toc')
%% Known Environment with A* and STL based Path Planning (second step)

% Setting up pre-requisities
model.hor_length = (size(model.astar_path,1)-1)/model.ds;
model.u0.min = model.u0.min(:,ones(1,model.hor_length+1)); % minimum control input ---> set in system description code
model.u0.max = model.u0.max(:,ones(1,model.hor_length+1)); % maximum control input

tic;
% MILP Problem solver
model = generate_STL_path(model,map,model.astar_path);
model.STL_Astar_path = model.solutions{1,3};

fprintf('MILP + A* solver elapsed time is: %.2f seconds. \n',toc')
%% Known Environment with RRT* and STL based Path Planning (second step)

% Setting up pre-requisities
model.hor_length = (size(model.rrtstar_path,1)-1)/model.ds;
model.u0.min = model.u0.min(:,ones(1,model.hor_length+1)); % minimum control input ---> set in system description code
model.u0.max = model.u0.max(:,ones(1,model.hor_length+1)); % maximum control input

tic;
% MILP Problem solver
model = generate_STL_path(model,map,model.rrtstar_path);
model.STL_RRT_path = model.solutions{1,3};

fprintf('MILP + RRT* solver elapsed time is: %.2f seconds. \n',toc')

%% Running the MPC simulation %%
close all
% model.movieName = input('Type in the name of the gif file');
% model.movieName = 2;
% model = MPC_plotting(model,map);


%% Plot all Paths %%
arrow_distance = 8;
figure(2)
show(map)
hold on
for ii = 1:size(model.goals,3)
    fill(model.goals(:,1,ii),model.goals(:,2,ii),[0 0.5 0]);
end

plot(model.init_pos(1,1),model.init_pos(1,2),'.','MarkerSize',30)

p1 = plot(model.STL_Astar_path(1,:),model.STL_Astar_path(2,:),'--k','LineWidth',1.2, 'DisplayName','A*'); 
p2 = plot(model.STL_RRT_path(1,:),model.STL_RRT_path(2,:),'--m','LineWidth',1.2,'DisplayName','RRT*');
p3 = plot(LTL_path(size(model.goals,3)+1:end,1),LTL_path(size(model.goals,3)+1:end,2),'b','LineWidth',1.2,'DisplayName','LTL');
p4 = plot(model.STL_path(1,:),model.STL_path(2,:),'r','LineWidth',1.2,'DisplayName','LTL + MILP');
legend([p1,p2,p3,p4])
title('Path Planners comparison (Simple Map)')

hold off


%% Get Robustness and Prediction Horizon %%
rho_req = 2;
dim = [0.6,0.8,0,0.1];
figure(3)
hold on;
[model.min_dis,model.min_rho] = get_robustness(model.LTL_path,map);     % STL spec min_d[t] > 1
r3 = plot(model.min_dis,'b','LineWidth',1.2,'DisplayName','LTL');
[model.min_dis,model.min_rho] = get_robustness(model.STL_path',map);     % STL spec min_d[t] > 1
r4 = plot(model.min_dis,'r','LineWidth',1.2,'DisplayName','LTL + MILP');


[model.min_dis,model.min_rho] = get_robustness(model.STL_Astar_path',map);     % STL spec min_d[t] > 1
r1 = plot(model.min_dis,'--k','LineWidth',1.2, 'DisplayName','A*');
[model.min_dis,model.min_rho] = get_robustness(model.STL_RRT_path',map);     % STL spec min_d[t] > 1
r2 = plot(model.min_dis,'--m','LineWidth',1.2,'DisplayName','RRT*');
r5 = plot(xlim, [1 1]*1, '--c','LineWidth',1.5,'DisplayName','Min safety \rho_{min}');
% plot(xlim, [0 0]*1, '--r','LineWidth',1.5)
legend([r1,r2,r3,r4,r5])
title('Safety Robustness')
if strcmp('simpleMap',mapName)
    xlim([0,90])
    ylabel('Robustness \rho(\phi_1)')
else
    xlim([0,275])
    ylabel('Robustness \rho(\phi_2)')
end
ylim([-1,15])
xlabel('Time Stamps (s)')


hold off


















