function [P_car_des,Pd_car_des]=Quad_Swing_traj(pha_sw)
%-----给定相位信号，计算摆动相足端规划轨迹-----%
global L_span T_st v_des Hip_Height delta Swing_Height 
t=pha_sw;

x0=-L_span;
xf=L_span;
xd0=-v_des;
xdf=-v_des;
z0=Hip_Height;
zm=Hip_Height+Swing_Height;
zf=Hip_Height;
zd0=delta*pi/T_st;
zdf=-delta*pi/T_st;

key_point_x=[x0;xf;xd0;xdf;0;0];
key_point_z=[z0;zm;zf;zd0;zdf];

tf=1;

 X = [
    0           0           0         0       0   1
    tf^5        tf^4        tf^3      tf^2    tf  1
    0           0           0         0       1   0
    5*tf^4      4*tf^3      3*tf^2    2*tf    1   0
    0           0           0         2       0   0
    20*tf^3     12*tf^2     6*tf      2       0   0
 ];
tz=1;tmz=0.5;
 Z = [
    0           0         0       0     1
    tmz^4       tmz^3     tmz^2   tmz   1
    tz^4        tz^3      tz^2    tz    1
    0           0         0       1     0
    4*tz^3      3*tz^2    2*tz    1     0
 ];
z_coeffs = (Z \ key_point_z)';%左除相当于inv(X)*key_point_y
x_coeffs = (X \ key_point_x)';

% coefficients of derivatives 
x_coeffs_d = x_coeffs(1:5) .* (5:-1:1);
z_coeffs_d = z_coeffs(1:4) .* (4:-1:1);
% coeffs_dd = coeffs_d(1:4) .* (4:-1:1);
 

    px=polyval(x_coeffs,t); 
    pz=polyval(z_coeffs,t);
    P_car_des=[px;pz];
    pdx = polyval(x_coeffs_d, t);
    pdz = polyval(z_coeffs_d, t);
    Pd_car_des=[pdx;pdz];
