function model = robot_system_description()
% SP = GET_SYSTEM_DESCRIPTION()
% initializes unicycle robot dynamics

 
  
  model.A = [0 0 1 0; 
    0 0 0 1;
    0 0 0 0;
    0 0 0 0];
  
  model.Bu = [0 0 1 0;0 0 0 1]';
  model.Bw = zeros(size(model.Bu));
  model.B = [model.Bu model.Bw];
  
  model.C = [1 0 0 0;
    0 1 0 0];
  
  model.Du = [0 0; 0 0]';
  model.Dw = zeros(size(model.Du));
  model.D = [model.Du model.Dw];
  
  model.u0.A = model.A;
  model.u0.B = model.B;
  %       SP.u0.Bu = SP.Bu;
  %       SP.u0.Bw = SP.Bw;
  model.u0.C = model.C;
  model.u0.D = model.D;
  %       SP.u0.Du = SP.Du;
  %       SP.u0.Dw = SP.Dw;
  
  model.u0.input_values_human = [];
  model.u0.min = -1*[1 1]';
  model.u0.max = 1*[1 1]';
  
  model.n = size(model.A,1);
  model.n_inputs = size(model.Bu,2);
  model.n_outputs = size(model.C,1);  
  
  % model.x0 = [2;2;0;0];
  
  model.observer_gains = 0.1*eye(3);
  model.ds = 1;
   
  
end