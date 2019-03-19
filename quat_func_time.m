function q=quat_func_time(t)
global alpha;
% q=[cos(t)*sin(alpha*(t-t^2)/2);sin(t)*sin(alpha*(t-t^2)/2);0;cos(alpha*(t-t^2)/2)];
q=(1/sqrt(2))*[sin(cos(alpha*t)); -cos(cos(alpha*t));-sin(sin(alpha*t)); cos(sin(alpha*t))];