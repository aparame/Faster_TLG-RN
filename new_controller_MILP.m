function controller = new_controller_MILP(model)
% controller = setup_MILP_controller_all_cons(model) encodes the MILP controller
% change objective function to involve minimizing velocity and MSE
% between trajectory and states
%%  define variables to be optimized
u = sdpvar(repmat(model.n_inputs,1, model.hor_length + 1),ones(1,model.hor_length + 1)); % control signal variable
x = sdpvar(repmat(model.n,1, model.hor_length+2),ones(1,model.hor_length+2)); % state space variable
y = sdpvar(repmat(model.n_outputs,1, model.hor_length + 1),ones(1,model.hor_length + 1)); % output variable
LTL_path = sdpvar(repmat(2,1, model.hor_length + 1),ones(1,model.hor_length + 1)); % LTL_path variable
w = sdpvar(repmat(model.n_inputs,1, model.hor_length + 1),ones(1,model.hor_length + 1));

x_cur = sdpvar(model.n,1); % current state of the system
rem_hor_length = sdpvar(1,1); % remaining horizon length


%% initialize constraints and objective
% Weights for cost Function %
Q_1 = 5;   % weight function for velocity
Q_2 = 0.4*eye(2);    % weight matrix for states
uVec = [u{:}];
objective = Q_1*sum(sum(abs(uVec)));    %objective function is to minimize the control inputs aka the robot velocity

constraints = [];

if isempty(model.u0.input_values_human)
    model.u0.input_values_human = zeros(size(u));
end

constraints = [constraints, x{1} == x_cur];


for k = 1:model.hor_length+1

    % add system dynamics as constraints
    constraints = [constraints, x{k+1} == model.tf_disc.A*x{k}+model.tf_disc.B*[u{k}; w{k}]];
    constraints = [constraints, y{k} == model.tf_disc.C*x{k}+model.tf_disc.D*[u{k}; w{k}]];

    % add bounds on control input signal as constraints
    constraints = [constraints, model.u0.min(:,k) <= u{k} <= model.u0.max(:,k)];
    constraints = [constraints, [0;0;-10000;-10000] <= x{k} <= [100;100;10000;10000] ];


    % set control value to zero for time-points beyond remaining horizon length
    if k > value(rem_hor_length)
        constraints = [constraints, u{k} == zeros(model.n_inputs,1)];
    end

    % safety robsutness constraint
    % 
    % for i = 1:12
    %     constraints = [constraints, A_map(i).vals*y{k} - b_map(i).vals >= -1];
    % end

    % Objective function to minimize contorl input and stay close to track

    objective = objective + norm(Q_2*(y{k} - LTL_path{k}),1);
end


% define what the controller inputs are
parameters_in = {x_cur, rem_hor_length,[LTL_path{:}]};

% define what the controller outputs are
solutions_out = {[u{:}], [x{:}], [y{:}]};


%% define settings

% verbose = 2 ---> gives detailed iteration wise results ---> good for debugging
% change solvername to use a different solver than gurobi
% if no solver option is provided yalmip uses the default in-built one

ops = sdpsettings('verbose', 2, 'solver', 'gurobi' );
controller = optimizer(constraints, objective, ops,parameters_in,solutions_out);


end