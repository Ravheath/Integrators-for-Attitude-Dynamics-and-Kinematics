function xdot=Dynamics(t,x)
global I;
q=x(1:4);
w=x(5:7);
q_dot=0.5*quat_prod([w;0],q);
w_dot=I\(torque(t)-cross(w,I*w));
xdot=[q_dot; w_dot];
