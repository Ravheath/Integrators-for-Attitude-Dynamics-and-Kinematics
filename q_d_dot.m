function q_d_dot=q_d_dot(t)
% this function gives q_double dot 
% global ALPHA;
% q_d_dot=-(1/sqrt(2))*(ALPHA^2)*[cos(ALPHA*t)*cos(cos(ALPHA*t)) + ((sin(ALPHA*t))^2)*sin(cos(ALPHA*t));
%                                 cos(ALPHA*t)*sin(cos(ALPHA*t)) - ((sin(ALPHA*t))^2)*cos(cos(ALPHA*t));
%                                 -sin(ALPHA*t)*cos(sin(ALPHA*t))- ((cos(ALPHA*t))^2)*sin(sin(ALPHA*t));
%                                 -sin(ALPHA*t)*sin(sin(ALPHA*t))+ ((cos(ALPHA*t))^2)*cos(sin(ALPHA*t))];
global beta
q_d_dot=beta*0.5*beta*0.5*[0;-sin(beta*t/2);0;-cos(beta*t/2)];