syms ct st cp sp cs ss

R1 = [cp -sp 0; sp cp 0; 0 0 1];
R2 = [ct 0 st; 0 1 0; -st 0 ct];
R3 = [1 0 0; 0 cs -ss; 0 ss cs];

R_231 = R1 * R3 * R2
R_231_adj = adjoint(R_231);
R_231_det = det(R_231);
R_231_inv = R_231_adj / R_231_det;