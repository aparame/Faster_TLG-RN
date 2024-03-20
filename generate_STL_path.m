function model = generate_STL_path(model,map)
% argvout = GENERATE_CRITICAL_CONSTRAINTS_MILP(model) identifies all the critical
% constraints that need to be activated for the known environment in an
% open-loop fashion and encodes the MILP controller

yalmip('clear')

%% Discretize the system
model.ss_cont= ss(model.A, model.B, model.C, model.D); % continuous dynamics
model.tf_disc = c2d(model.ss_cont, model.ds); % discrete dynamics

%% Setup the controller and the optimization process

model.controller = new_controller_MILP(model);

x_cur = model.x0;
model.u0.values = zeros(model.n_inputs, model.hor_length + 1); % optimized control signal values to be stored here

%% Initialize other variables
model.t0 = 0; % initial time

sol_count = 1; % keeps track of how many iterations were required to solve the problem


% set all constraints to be inactive ---> they are made active as required

map_matrix = (getOccupancy(map)*100);
map_matrix = map_matrix > 0.5;
sdm = signedDistanceMap(map_matrix,InterpolationMethod="none");
rho_req = 2; %minimum robustness requirement
% path_rho = 1*ones(1,model.hor_length + 1);
%% Start the main open-loop optimization procedurel

% while (min(path_rho) <= rho_req)
  
%===========================================================================
% Compute Robustness and identify the critical constraint
%===========================================================================

% solve the optimization problem
[solutions,diagnostics] = model.controller{{x_cur(:),model.hor_length+1,model.LTL_path'}};


% proceed if feasible
if diagnostics == 1 || diagnostics == 12 || diagnostics == 15
    error('The problem is infeasible');
end
model.u0.values = solutions{1};
model.yout_disc = solutions{3}';

%===========================================================================
% Compute Robustness and identify the critical constraint
%===========================================================================

% compute critical time, predicate and robustness

% path_rho = [current_rho(model.yout_disc,sdm,rho_req)]';

sol_count = sol_count + 1;
  
% end

%% Output results
model.solutions = solutions;


end


