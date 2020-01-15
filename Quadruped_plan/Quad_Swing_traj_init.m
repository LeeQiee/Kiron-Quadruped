function [P_car_des,Pd_car_des]=Quad_Swing_traj_init(pha_sw)
%-----初始阶段摆动相轨迹规划，给定相位信号，计算摆动相足端规划轨迹-----%
global T_st Hip_Height delta Swing_Height
t=pha_sw;

x0=0;
xm=0;
xd0=0;
xdm=0;
z0=Hip_Height;
zm=Hip_Height+Swing_Height;
zd0=delta*pi/T_st;
zdm=0;

key_point_x=[x0;xm;xd0;xdm;0;0];
key_point_z=[z0;zm;zd0;zdm;0;0];

tf=1;

 X = [
    0           0           0         0       0   1
    tf^5        tf^4        tf^3      tf^2    tf  1
    0           0           0         0       1   0
    5*tf^4      4*tf^3      3*tf^2    2*tf    1   0
    0           0           0         2       0   0
    20*tf^3     12*tf^2     6*tf      2       0   0
 ];
z_coeffs = (X \ key_point_z)';%左除相当于inv(X)*key_point_y
x_coeffs = (X \ key_point_x)';

% coefficients of derivatives 
x_coeffs_d = x_coeffs(1:5) .* (5:-1:1);
z_coeffs_d = z_coeffs(1:5) .* (5:-1:1);
% coeffs_dd = coeffs_d(1:4) .* (4:-1:1);
 
    px=polyval(x_coeffs,t); 
    pz=polyval(z_coeffs,t);
    P_car_des=[px;pz];
    pdx = polyval(x_coeffs_d, t);
    pdz = polyval(z_coeffs_d, t);
    Pd_car_des=[pdx;pdz];