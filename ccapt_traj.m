function [desired_state, bumped_starts, bumped_goals] = ccapt_traj(starts, goals, R, vmax, qn, t)
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
            goal_dists = squareform(pdist(goals))+5*R*eye(size(goals,1));
        end
    end
    
    bumped_starts = starts;
    bumped_goals = goals;
    bumped_starts(:,3) = bumped_starts(:,3)*4;
    bumped_goals(:,3) = bumped_goals(:,3)*4;

    % sanity checks
    N = size(starts, 1);
    n = size(starts, 2);
    if size(goals, 1) ~= N
        warning('Different numbers of start and goal points!');
    end
    if size(goals, 2) ~= n
        error('Different dimensions in start and goal space!');
    end

    
    alphas = zeros(8,0);
    tf = [];
    i = 0;
    starts(:,3) = starts(:,3)*4;
    goals(:,3) = goals(:,3)*4;
    while size(goals,1) > 0
        i = i + 1;

        % construct D
        D = pdist2(bsxfun(@times, starts, [1 1 0.25]), bsxfun(@times, goals, [1 1 0.25])).^2;

        % solve assignment problem
        A = assignmentoptimal(D)';

        % trajectories!

        X{i} = reshape(starts', [N*n 1]);
        newgoals = zeros(size(goals));
        newgoals(A~=0,:) = goals(A(A~=0),:); % reorder goals for robots who are moving
        newgoals(A==0,:) = starts(A==0,:);   % non-moving robots stay at their start points
        G{i} = reshape(newgoals',  [numel(newgoals) 1]);
        
        D = pdist2(starts, goals).^2;
        tf(i) = sqrt(max(D(sub2ind(size(D), 1:nnz(A), A(A~=0)))))/vmax;
        
        % remove solved goals
        starts(A~=0,:) = goals(A(A~=0),:);
        goals(A(A~=0),:) = [];
        
        
        alphas(:,i) = [  1 0 0 0 0 0 0 0;
                         0 1 0 0 0 0 0 0;
                         0 0 2 0 0 0 0 0;
                         0 0 0 6 0 0 0 0;
                         1   tf(i)  tf(i)^2    tf(i)^3    tf(i)^4    tf(i)^5    tf(i)^6     tf(i)^7;
                         0   1      2*tf(i)    3*tf(i)^2  4*tf(i)^3  5*tf(i)^4  6*tf(i)^5   7*tf(i)^6;
                         0   0      2          6*tf(i)    12*tf(i)^2 20*tf(i)^3 30*tf(i)^4  42*tf(i)^5;
                         0   0      0          6          24*tf(i)   60*tf(i)^2 120*tf(i)^3 210*tf(i)^4]\[0 0 0 0 1 0 0 0]';
        if i > 1
            tf(i) = tf(i) + tf(i-1);
        end

        alphas(:,i) = flipud(alphas(:,i));
        
    end
    
    desired_state = [];
    return;
end

if t < tf(end)
    for i=1:length(tf)
        if t < tf(i)
            tt = t;
            if i > 1
                tt = t - tf(i-1);
            end
            pos = X{i}((1:3) + (qn-1)*3) + polyval(alphas(:,i), tt)*(G{i}((1:3) + (qn-1)*3)-X{i}((1:3) + (qn-1)*3));
            vel = polyval(polyder(alphas(:,i)), tt)*(G{i}((1:3) + (qn-1)*3)-X{i}((1:3) + (qn-1)*3));
            acc = polyval(polyder(polyder(alphas(:,i))), tt)*(G{i}((1:3) + (qn-1)*3)-X{i}((1:3) + (qn-1)*3));
            break;
        end
    end
else
    pos = G{end}((1:3) + (qn-1)*3);
    vel = [0;0;0];
    acc = [0;0;0]; 
end

desired_state.pos = pos(:);
desired_state.vel = vel(:);
desired_state.acc = acc(:);
desired_state.yaw = 0;
desired_state.yawdot = 0;

