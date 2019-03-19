function q_dot=quat_deriv_func_time(t)
% this function gives the analytical trajectory for the tangent vector on
% the manifold corresponding to the trajectory defined in quat_func_time.m
% quaternion
global alpha; % parameter for the quaternion trajectory 
% beta=alpha*0.5;  
% q=[-sin(t)*sin(alpha*(t-t^2)/2)+cos(t)*beta*(1-2*t)*cos(alpha*(t-t^2)/2);
%     cos(t)*sin(alpha*(t-t^2)/2)+sin(t)*beta*(1-2*t)*cos(alpha*(t-t^2)/2);
%     0;
%     -beta*(1-2*t)*sin(alpha*(t-t^2)/2)];
q_dot=(1/sqrt(2))*alpha*[-sin(alpha*t)*cos(cos(alpha*t));
                         -sin(alpha*t)*sin(cos(alpha*t));
                         -cos(alpha*t)*cos(sin(alpha*t));
                          -cos(alpha*t)*sin(sin(alpha*t))];
