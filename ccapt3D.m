function [traj, vel, acc] = ccapt3D(starts, goals, R, vmax, dt)
% starts, goals: each row is a point
    
    start_dists = pdist(starts);
    if any(start_dists(:) < 2*R)
        error('Some start points too close together!')
    end

    goal_dists = pdist(goals);
    if any(goal_dists(:) < 2*R)
        error('Some goal points too close together!')
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
                
    beta = polyval(flipud(alphas), (0:dt:tf));
    beta_vel = polyval(polyder(flipud(alphas)), (0:dt:tf));
    beta_acc = polyval(polyder(polyder(flipud(alphas))), (0:dt:tf));
    
    if beta(end)~=1;beta(end+1)=1;end;
    X = reshape(starts', [N*n 1]);
    G = reshape(goals(assignments,:)',  [N*n 1]);
    traj = bsxfun(@plus, X, bsxfun(@mtimes, beta, G - X));
    vel  = bsxfun(@plus, 0, bsxfun(@mtimes, beta_vel, G - X));
    acc  = bsxfun(@plus, 0, bsxfun(@mtimes, beta_acc, G - X));
end

