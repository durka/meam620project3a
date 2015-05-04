%% phase 1

N = [10 50 100:100:1000];
n = 10;
R = 0.05;
vmax = 1;
dt = 0.01;

timings = zeros(length(N), n);
gamma = cell(size(N));
starts = cell(size(N));
goals = cell(size(N));
for i=1:length(N)
    fprintf('Timing CCAPT-2D for %d robots...\n', N(i));
    
    % move the starts and goals
    [gamma{i}, starts{i}, goals{i}] = ccapt(rand(N(i), 2)*100, rand(N(i), 2)*100, R, vmax, dt);
    
    % average timings
    for j=1:n
        fprintf('\t%d\n', j);
        tic;
        ccapt(starts{i}, goals{i}, R, vmax, dt);
        timings(i,j) = toc;
    end
end

loglog(N, mean(timings, 2));
grid on;
xlabel('Number of robots');
ylabel('Planning time (s)');
title('C-CAPT 2D Planning performance');
print -depsc timing_phase1

save timing_phase1 N n R vmax dt gamma starts goals timings

