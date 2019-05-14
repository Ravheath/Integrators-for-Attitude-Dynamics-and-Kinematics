function q_dot=quat_deriv_func_time(t)
% this function gives the analytical trajectory for the tangent vector on
% the manifold corresponding to the trajectory defined in quat_func_time.m
% quaternion
% global ALPHA; % parameter for the quaternion trajectory 
% % beta=ALPHA*0.5;  
% % q=[-sin(t)*sin(ALPHA*(t-t^2)/2)+cos(t)*beta*(1-2*t)*cos(ALPHA*(t-t^2)/2);
% %     cos(t)*sin(ALPHA*(t-t^2)/2)+sin(t)*beta*(1-2*t)*cos(ALPHA*(t-t^2)/2);
% %     0;
% %     -beta*(1-2*t)*sin(ALPHA*(t-t^2)/2)];
% q_dot=(1/sqrt(2))*ALPHA*[-sin(ALPHA*t)*cos(cos(ALPHA*t));
%                          -sin(ALPHA*t)*sin(cos(ALPHA*t));
%                          -cos(ALPHA*t)*cos(sin(ALPHA*t));
%                           -cos(ALPHA*t)*sin(sin(ALPHA*t))];
global beta
q_dot=beta*0.5*[0;cos(beta*t/2);0;-sin(beta*t/2)];