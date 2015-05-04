%% pool table

cue = [5 19
      -5 7
      15 7];
basket = [ 5 1
          15 7
          -5 7];
balls = [5 11
         3 9; 5 9; 7 9
         1 7; 3 7; 5 7; 7 7; 9 7
         3 5; 5 5; 7 5
         5 3];
goals = [basket; balls];
goals(2:end,1) = goals(2:end,1) + 1;
starts = [cue; balls];
dcapt2d(starts, goals, 0.25, 1, 5, 0.1, true);
