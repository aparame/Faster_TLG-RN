function astar_path_ex = AStar(model,map)
goals = model.goals;
init_pos = model.init_pos;

init_pos_a_star = round([init_pos(:,1),map.GridSize(1,2)-init_pos(:,2)]);
for i=1:size(goals,3)
    goals_poly = polyshape(goals(:,1,i),goals(:,2,i));
    [goal_centre_x,goal_centre_y] = centroid(goals_poly);
    goals_a_star(:,:,i) = round([goal_centre_x,map.GridSize(1,2)-goal_centre_y]);
    if i==1
        astar = plannerAStarGrid(map);
        astar_path_ex = plan(astar,flip(init_pos_a_star),flip(goals_a_star(:,:,i)));

    else
        astar = plannerAStarGrid(map);
        astar_path_ex = [astar_path_ex;plan(astar,flip(goals_a_star(:,:,i-1)),flip(goals_a_star(:,:,i)))];

    end
end

end