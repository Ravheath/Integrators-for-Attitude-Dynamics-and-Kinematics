function w_dot=Dynamics2(t,x)
global I;
w_dot=I\(torque(t)-cross(x,I*x));
