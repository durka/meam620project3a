function [desired_state] = diamond(t, qn)
% DIAMOND trajectory generator for a diamond

% =================== Your code goes here ===================
% You have to set the pos, vel, acc, yaw and yawdot variables
s = 1.7;

if 0<=t && t<=s
    pos = 1/s*[.25*t;t*sqrt(2); t*sqrt(2)];
    vel = 1/s*[.25; sqrt(2); sqrt(2)];
    acc = [0; 0; 0];
    yaw = 0;
    yawdot = 0;
end

if s<t && t<=2*s
    pos = 1/s*[(.5/2)*t;(2*s - t)*sqrt(2); (2*s - t)*sqrt(2)+(t-s)*2*sqrt(2)];
    vel = 1/s*[.5/2; -sqrt(2); -sqrt(2)+2*sqrt(2)];
    acc = [0; 0; 0];
    yaw = 0;
    yawdot = 0;
end

if 2*s<t && t<=3*s
    pos = 1/s*[.75*t/3; -(t - 2*s)*sqrt(2); (3*s - t)*2*sqrt(2) + (t - 2*s)*sqrt(2)];
    vel = 1/s*[.75/3; -sqrt(2); -sqrt(2)];
    acc = [0; 0; 0];
    yaw = 0;
    yawdot = 0;
end

if 3*s<t && t<4*s
    pos = 1/s*[t/4; -(4*s - t)*sqrt(2); (4*s - t)*sqrt(2)];
    vel = 1/s*[1/4; sqrt(2); -sqrt(2)];
    acc = [0; 0; 0];
    yaw = 0;
    yawdot = 0;
end

if t>=4*s
    pos = [1,0,0];
    vel = [0; 0; 0];
    acc = [0; 0; 0];
    yaw = 0;
    yawdot = 0;
end
% =================== Your code ends here ===================

desired_state.pos = pos(:);
desired_state.vel = vel(:);
desired_state.acc = acc(:);
desired_state.yaw = yaw;
desired_state.yawdot = yawdot;

end
