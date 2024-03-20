function rrtstar_path = RRT_star(model,map,mapName)
ss = stateSpaceSE2;
sv = validatorOccupancyMap(ss);
sv.Map = map;
sv.ValidationDistance = 0.1;
ss.StateBounds = [map.XWorldLimits; map.YWorldLimits; [-pi pi]];
goals = flip(model.goals,3);
init_pos = model.init_pos;
if strcmp('simpleMap',mapName)
    init_pos_rrt_star = round([init_pos(:,1),init_pos(:,2),0]);
    for i=1:size(goals,3)
        goals_poly = polyshape(goals(:,1,i),goals(:,2,i));
        [goal_centre_x,goal_centre_y] = centroid(goals_poly);
        goals_rrt_star(:,:,i) = round([goal_centre_x,goal_centre_y,0]);

        if i==1
            planner = plannerRRTStar(ss,sv, ContinueAfterGoalReached=true, MaxConnectionDistance=0.5);
            rng(100,'twister') % repeatable result
            [rrtstar, ~] = plan(planner,init_pos_rrt_star,goals_rrt_star(:,:,i));
            rrtstar_path = rrtstar.States(:,1:2);
        else
            [rrtstar, ~] = plan(planner,goals_rrt_star(:,:,i-1),goals_rrt_star(:,:,i));
            rrtstar_path = [rrtstar_path;rrtstar.States(:,1:2)];
        end
    end
elseif strcmp('complexMap',mapName)
    init_pos_rrt_star = round([init_pos(:,1),init_pos(:,2),0]);
    for i=1:size(goals,3)
        goals_poly = polyshape(goals(:,1,i),goals(:,2,i));
        [goal_centre_x,goal_centre_y] = centroid(goals_poly);
        goals_rrt_star(:,:,i) = round([goal_centre_x,goal_centre_y,0]);

        if i==1
            planner = plannerRRTStar(ss,sv,ContinueAfterGoalReached=true, MaxConnectionDistance=0.5);
            rng(100,'twister') % repeatable result
            [rrtstar, ~] = plan(planner,init_pos_rrt_star,goals_rrt_star(:,:,i));
            rrtstar_path = rrtstar.States(:,1:2);
        else
            [rrtstar, ~] = plan(planner,goals_rrt_star(:,:,i-1),goals_rrt_star(:,:,i));
            rrtstar_path = [rrtstar_path;rrtstar.States(:,1:2)];
        end
    end
end

end