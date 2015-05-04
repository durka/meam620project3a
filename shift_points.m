function [starts, goals] = shift_points(starts, goals, R)
    start_dists = squareform(pdist(starts))+5*R*eye(size(starts,1));
    if any(start_dists(:) < 2*sqrt(2)*R)
        disp('Some start points too close together! Shifting points')
        while any(start_dists(:) < 2*sqrt(2)*R)
            [p1, p2] = find(start_dists < 2*sqrt(2)*R,1);
            starts(p2,:) = starts(p1,:) + 5*R*(starts(p2,:) - starts(p1,:))/norm(starts(p2,:) - starts(p1,:));
            start_dists = squareform(pdist(starts))+5*R*eye(size(starts,1));
        end
    end

    goal_dists = squareform(pdist(goals))+5*R*eye(size(goals,1));
    if any(goal_dists(:) < 2*sqrt(2)*R)
        disp('Some goal points too close together! Shifting points')
        while any(goal_dists(:) < 2*sqrt(2)*R)
            [p1, p2] = find(goal_dists < 2*sqrt(2)*R,1);
            goals(p2,:) = goals(p1,:) + 5*R*(goals(p2,:) - goals(p1,:))/norm(goals(p2,:) - goals(p1,:));
            goal_dists = squareform(pdist(goals))+5*R*eye(size(goals,1));
        end
    end
    end