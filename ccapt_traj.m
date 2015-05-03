function [desired_state, starts, goals] = ccapt_traj(starts, goals, R, vmax, qn, t)
% starts, goals: each row is a point

persistent X G alphas tf;

if ~isempty(starts)
    starts(:,3) = starts(:,3)/4;
    start_dists = squareform(pdist(starts))+5*R*eye(size(starts,1));
    if any(start_dists(:) < 2*sqrt(2)*R)
        disp('Some start points too close together! Shifting points')
        while any(start_dists(:) < 2*sqrt(2)*R)
            [p1, p2] = find(start_dists < 2*sqrt(2)*R,1);
            starts(p2,:) = starts(p1,:) + 5*R*(starts(p2,:) - starts(p1,:))/norm(starts(p2,:) - starts(p1,:));
            start_dists = squareform(pdist(starts))+5*R*eye(size(starts,1));
        end
    end

    goals(:,3) = goals(:,3)/4;
    goal_dists = squareform(pdist(goals))+5*R*eye(size(goals,1));
    
    if any(goal_dists(:) < 2*sqrt(2)*R)
        disp('Some goal points too close together! Shifting points')
        while any(goal_dists(:) < 2*sqrt(2)*R)
            [p1, p2] = find(goal_dists < 2*sqrt(2)*R,1);
            goals(p2,:) = goals(p1,:) + 5*R*(goals(p2,:) - goals(p1,:))/norm(goals(p2,:) - goals(p1,:));
            goal_dists = squareform(pdist(goals))+5*eye(size(goals,1));
        end
    end

    % sanity checks
    N = size(starts, 1);
    n = size(starts, 2);
    if size(goals, 1) ~= N
        error('Different numbers of start and goal points!');
    end
    if size(goals, 2) ~= n
        error('Different dimensions in start and goal space!');
    end

    % construct D
    D = pdist2(starts, goals).^2;
    starts(:,3) = starts(:,3)*4;
    goals(:,3) = goals(:,3)*4;

    % solve assignment problem
    assignments = assignmentoptimal(D)';

    % trajectories!

    tf = sqrt(max(D(sub2ind(size(D), 1:N, assignments))))/vmax;
    alphas = [  1 0 0 0 0 0 0 0;
                0 1 0 0 0 0 0 0;
                0 0 2 0 0 0 0 0;
                0 0 0 6 0 0 0 0;
                1   tf  tf^2    tf^3    tf^4    tf^5    tf^6    tf^7;
                0   1   2*tf    3*tf^2  4*tf^3  5*tf^4  6*tf^5  7*tf^6;
                0   0   2       6*tf    12*tf^2 20*tf^3 30*tf^4 42*tf^5;
                0   0   0       6       24*tf   60*tf^2 120*tf^3 210*tf^4]\[0 0 0 0 1 0 0 0]';
    
    alphas = flipud(alphas);
    
    X = reshape(starts', [N*n 1]);
    G = reshape(goals(assignments,:)',  [N*n 1]);
    desired_state = [];
    return;
end
if t < tf
    pos = X((1:3) + (qn-1)*3) + polyval(alphas, t)*(G((1:3) + (qn-1)*3)-X((1:3) + (qn-1)*3));
    vel = polyval(polyder(alphas), t)*(G((1:3) + (qn-1)*3)-X((1:3) + (qn-1)*3));
    acc = polyval(polyder(polyder(alphas)), t)*(G((1:3) + (qn-1)*3)-X((1:3) + (qn-1)*3));
else
    pos = G((1:3) + (qn-1)*3);
    vel = [0;0;0];
    acc = [0;0;0]; 
end

desired_state.pos = pos(:);
desired_state.vel = vel(:);
desired_state.acc = acc(:);
desired_state.yaw = 0;
desired_state.yawdot = 0;

