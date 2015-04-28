function gamma = ccapt(starts, goals, R, vmax, dt)
% starts, goals: each row is a point

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
    X = reshape(starts', [N*n 1]);
    G = reshape(goals',  [N*n 1]);
    D = pdist2(starts, goals).^2;

    % solve assignment problem
    assignments = lapjv(D);
    phi = eye(N);
    phi = phi(assignments,:);
    Phi = kron(phi, eye(n));

    % trajectories!
    tf = 0;
    for i=1:N
        t = sqrt(D(i,assignments(i)))/vmax;
        if t > tf
            tf = t;
        end
    end
    alpha = 1/tf;
    beta = alpha * (0:dt:tf);
    gamma = bsxfun(@mtimes, (1 - beta), X) + bsxfun(@mtimes, beta, (Phi*G + (eye(N*n) - Phi*Phi')*X));

end

