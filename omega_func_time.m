function omega=omega_func_time(t)
% w=2*quat_prod(quat_deriv_func_time(t),quat_inv(quat_func_time(t)));
% omega=w(1:3,1);
global beta
omega=[0,beta,0]';