function q_prod=quat_prod_onenorm(p,q)
% this product uses the JPL convention prefered by Markley
p_v=p(1:3,1);
q_v=q(1:3,1);
p_s=p(4);
q_s=q(4);
r=[p_s*q_v+q_s*p_v-cross(p_v,q_v);p_s*q_s-dot(p_v,q_v)];
q_prod=r/norm(r);
