function [desired_state] = circle(t, qn)
% CIRCLE trajectory generator for a circle

% =================== Your code goes here ===================
% You have to set the pos, vel, acc, yaw and yawdot variables
speed = .125;
if 0<=t<8
    pos = [5*cos(2*pi*speed*t);5*sin(2*pi*speed*t); 2.5*speed*t];
    vel = [-10*pi*speed*sin(2*pi*t*speed); 10*speed*pi*cos(2*pi*t*speed); 2.5*speed];
    acc = [-20*speed^2*pi*cos(2*pi*t*speed); -20*speed^2*pi*sin(2*pi*t*speed); 0];
    yaw = 0;
    yawdot = 0;
end

if t>=8
    pos = [5,0,2.5];
    vel = zeros(1,3);
    acc = zeros(1,3);
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
