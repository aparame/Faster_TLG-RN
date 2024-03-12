function [min_dis, min_rho] = get_robustness(model_path,map)
map_matrix = (getOccupancy(map)*100);
map_matrix = map_matrix > 0.5;
sdm = signedDistanceMap(map_matrix,InterpolationMethod="none");
min_dis = distance(sdm,model_path);

Bdata = BreachTraceSystem({'min_d'});
time_array = (1:1:height(min_dis))';
% min_d = reshape(min_d,[1,length(model.astar_path)]);
trace = [time_array,min_dis];
Bdata.AddTrace(trace);
phi = STL_Formula('phi','alw (min_d[t] > 1)');
Rphi = BreachRequirement(phi);
min_rho = Rphi.Eval(Bdata);


% Plotting %
figure(3)
dim = [0.6,0.8,0,0.1];
plot(min_dis,'LineWidth',1.5)
% ylim([-0.5,2]);
hold on
plot(xlim, [1 1]*1, '--k','LineWidth',1.5)
plot(xlim, [0 0]*1, '--r','LineWidth',1.5)
str = sprintf('Min Robustness: %.4f', min_rho);
annotation('textbox',dim,'String',str,'FitBoxToText','on');
title('Safety Robustness of STL Path')

xlabel('Robustness')
ylabel('Time Stamps (s)')


hold off