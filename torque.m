function tau=torque(t)
% this function gives the torque in the body frame 
% it should be noted that the torque is purely a function of time and this
% torque is in the "to be body frame ideally" i.e. the body frame
% characterized by the assumed quaternion trajectory 
%global I;  
%w=omega_func_time(t);
%q=quat_func_time(t);
%Omega_dot= q_d_dot(t)-0.25*quat_prod([w;0],quat_prod([w;0],q));
%Omega_dot=2*quat_prod(Omega_dot,quat_inv(q));
%Omega_dot=Omega_dot(1:3,1);
%tau= cross(w,I*w)+I*Omega_dot;
tau=5e-7*[sin((10e-4)*t);cos(sin((10e-3)*t));-sin(cos((10e-3)*t))];
