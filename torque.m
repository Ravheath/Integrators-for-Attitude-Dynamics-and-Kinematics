function tau=torque(t)
global I;  
w=omega_func_time(t);
q=quat_func_time(t);
Omega_dot= q_d_dot(t)-0.25*quat_prod([w;0],quat_prod([w;0],q));
Omega_dot=2*quat_prod(Omega_dot,quat_inv(q));
Omega_dot=Omega_dot(1:3,1);
tau= cross(w,I*w)+I*Omega_dot;