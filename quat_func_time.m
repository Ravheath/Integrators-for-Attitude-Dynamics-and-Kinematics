function q=quat_func_time(t)
global ALPHA;
% q=[cos(t)*sin(ALPHA*(t-t^2)/2);sin(t)*sin(ALPHA*(t-t^2)/2);0;cos(ALPHA*(t-t^2)/2)];
q=(1/sqrt(2))*[sin(cos(ALPHA*t)); -cos(cos(ALPHA*t));-sin(sin(ALPHA*t)); cos(sin(ALPHA*t))];