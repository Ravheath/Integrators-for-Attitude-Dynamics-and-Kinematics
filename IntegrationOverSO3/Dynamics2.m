function w_dot=Dynamics2(t,x)
%this function is used for new method of integration
%this function is called by rk42.m
global I; % moment of inertia matrix declared in main code
w_dot=I\(torque(t)-cross(x,I*x)); % calculating the derivative of omega
