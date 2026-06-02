%symbolically solve for equations based on G_T_B -> B_T_G calculations
syms ap X_o Y_o Z_o L0

B_T_G = [cos(ap) sin(ap) 0 -L0; -sin(ap) cos(ap) 0 0; 0 0 1 0; 0 0 0 1;]
G_R = [X_o; Y_o; Z_o; 1;]

B_R_G = B_T_G*G_R

%%
X_o*cos(ap) - L0 + Y_o*sin(ap)
     Y_o*cos(ap) - X_o*sin(ap)
                           Z_o
                             1

%%