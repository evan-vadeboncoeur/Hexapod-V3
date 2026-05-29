%symbolically solve for FK based on unknown parameters
syms a1 a2 a3 d1 o1 o2 o3 t1 t2 t3 ap1 ap2 ap3

a = [a1, a2, a3];
ap = [ap1, ap2, ap3];
d = [d1, 0, 0];
o = [o1, o2, o3];
t = [t1, t2, t3];

CURR_HTF = eye(4,4);
PREV=eye(4,4);

for i=3:-1:1
    CURR_HTF = [cos(t(i)), -sin(t(i))*cos(ap(i)), sin(t(i))*sin(ap(i)),  a(i)*cos(t(i));
                sin(t(i)), cos(t(i))*cos(ap(i)),  -cos(t(i))*sin(ap(i)), a(i)*sin(t(i));
                0,         sin(ap(i)),            cos(ap(i)),            d(i);
                0,         0,                     0,                     1;];
    PREV = CURR_HTF * PREV;
    
end

FK = PREV;

%solve FK with known parameters
%% Forward Kinematics for Hexapod Leg, RHS (Robot's LHS)
a_1 = 70; %[mm]
a_2 = 100; 
a_3 = 150; 
d_1 = 0;
d_2 = 0;
d_3 = 0;
o_1 = 0; %[deg] - figure out 'home' angles
o_2 = 0;%-pi/4;
o_3 = 0; %pi/4;
ap_1 = -pi/2;
ap_2 = 0;
ap_3 = 0;

%create arrays of each parameter to iterate
a_ = [a_1, a_2, a_3]; % a 
d_ = [d_1, d_2, d_3]; % d 
o_ = [o_1, o_2, o_3]; % theta offset
ap_ = [ap_1, ap_2, ap_3]; % alpha
t_ = [t1 + o_1, t2 + o_2, t3 + o_3]; % theta

C_HTF = eye(4,4);
P_HTF = sym(eye(4,4));

for i=3:-1:1
    C_HTF = [cos(t_(i)), -sin(t_(i))*cos(ap_(i)), sin(t_(i))*sin(ap_(i)),  a_(i)*cos(t_(i));
                sin(t_(i)), cos(t_(i))*cos(ap_(i)),  -cos(t_(i))*sin(ap_(i)), a_(i)*sin(t_(i));
                0,         sin(ap_(i)),            cos(ap_(i)),            d_(i);
                0,         0,                     0,                     1;];
    P_HTF = C_HTF * P_HTF;
    
end

FK_S = P_HTF;
FK_S = simplify(FK_S, 'IgnoreAnalyticConstraints', true);
FK_S = vpa(FK_S, 6);
FK_S = simplify(FK_S, 'IgnoreAnalyticConstraints', true)
%FK_S = combine(FK_S) % Tries to merge fractions and similar terms
%FK_S = double(vpa(FK_S, 6));  % Convert symbolic to numerical
% tolerance = 1e-10;  
% FK_S(abs(FK_S) < tolerance) = 0;  % Zero out small values
 
[cos(t2 + t3)*cos(t1), - 1.0*sin(t2 + t3)*cos(t1), -1.0*sin(t1), 70.0*cos(t1) + cos(t1)*(150.0*cos(t2 + t3) + 100.0*cos(t2))]
[cos(t2 + t3)*sin(t1), - 1.0*sin(t2 + t3)*sin(t1), cos(t1), 70.0*sin(t1) + sin(t1)*(150.0*cos(t2 + t3) + 100.0*cos(t2))]
[-1.0*sin(t2 + t3), -1.0*cos(t2 + t3), 0,   - 150.0*sin(t2 + t3) - 100.0*sin(t2)]
[ 0, 0, 0, 1.0]
                                                                                                                                                                                 

% 3 x 3 rotation matrix
R1_1 = cos(t2 + t3)*cos(t1);
R1_2 = - 1.0*sin(t2 + t3)*cos(t1);
R1_3 = -1.0*sin(t1);
R2_1 = cos(t2 + t3)*sin(t1);
R2_2 = - 1.0*sin(t2 + t3)*sin(t1);
R2_3 = cos(t1);
R3_1 = -1.0*sin(t2 + t3);
R3_2 = -1.0*cos(t2 + t3);
R3_3 = 0.0;
% 3 x 1 positon vector of EE frame represented in global frame
R1_4 = 70.0*cos(t1) + cos(t1)*(150.0*cos(t2 + t3) + 100.0*cos(t2)); % d_x
R2_4 = 70.0*sin(t1) + sin(t1)*(150.0*cos(t2 + t3) + 100.0*cos(t2)); % d_y
R3_4 = -150.0*sin(t2 + t3) - 100.0*sin(t2); %d_z
