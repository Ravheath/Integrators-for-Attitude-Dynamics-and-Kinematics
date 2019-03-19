function q_d_dot=q_d_dot(t)
% this function gives q_double dot 
global alpha;
q_d_dot=-(1/sqrt(2))*(alpha^2)*[cos(alpha*t)*cos(cos(alpha*t)) + ((sin(alpha*t))^2)*sin(cos(alpha*t));
                                cos(alpha*t)*sin(cos(alpha*t)) - ((sin(alpha*t))^2)*cos(cos(alpha*t));
                                -sin(alpha*t)*cos(sin(alpha*t))- ((cos(alpha*t))^2)*sin(sin(alpha*t));
                                -sin(alpha*t)*sin(sin(alpha*t))+ ((cos(alpha*t))^2)*cos(sin(alpha*t))];
