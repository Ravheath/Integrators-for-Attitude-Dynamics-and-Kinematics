function R_new=brute_force_orth(R)
K=R'*R;
R_tilde=chol(inv(K),'lower');
R_new=R*R_tilde;


