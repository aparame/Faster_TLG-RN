function model = MPC_plotting(model,map)


%% Initializing some variables %%
model.states_system = [model.x0(1:2);0];
tIndxs = 1:model.hor_length+1; % time indices corresponding to horizon length
model.u0.values = zeros(model.n_inputs, model.hor_length + 1);

%% Developing the initial car position on plot %%

hfig = figure;
hold on
figure(hfig);
show(map)

for ii = 1:size(model.goals,3)
    fill(model.goals(:,1,ii),model.goals(:,2,ii),[0 0.5 0]);
end
for ii = 1:model.n_obs
    fill(model.obs(:,1,ii),model.obs(:,2,ii),[0 0 0]);
end
car_sz = 2;
car = create_and_draw_car([model.states_system(1:2);0], eye(3), 0 , 1, car_sz);
time_disp=text(35,50,0,'Time taken = 0.000 sec','Color','b', 'FontSize', 12);

trail_length = 40;
car = create_display_vars_car(car, trail_length);

model.predicted_traj_handle = plot(model.STL_path(1), model.STL_path(2), ...
    'r', 'LineWidth', 2);

f = getframe;
[im,map] = rgb2ind(f.cdata,256,'nodither');


x_cur = model.solutions{1,2};
path = flip(model.solutions{3},2);

for ii = 1:model.hor_length + 1
    model.tIndxs = tIndxs;

    figure(hfig);
    car.z = [x_cur(1:2,ii);0];
    car.th = atan2(x_cur(4,ii), x_cur(3,ii));

    [car, time_disp] = update_car_paths(car, time_disp, ii, model.ds, trail_length);
    set(model.predicted_traj_handle, 'XData', path(1,1:size(tIndxs,2)), ...
        'YData', path(2,1:size(tIndxs,2)));


    drawnow;

    pause(0.1)
    f=getframe;
    im(:,:,1,ii) = rgb2ind(f.cdata,map,'nodither');


    tIndxs(end) = [];

end



imwrite(im,map,'SAE_2.gif','DelayTime', 2*model.ds,'LoopCount',inf);
model.im = im;
model.map = map;



end
