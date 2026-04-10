function populate_traj_planner(mdl)
% Set the trajectory planner MATLAB Function block script from algorithms/.
    embed_algorithm([mdl '/TrajPlanner'], 'traj_planner');
end
