function [P_des,Pd_des]=Swing_tra(pha_sw)
%-----给定相位信号，计算摆动相足端规划轨迹-----%
global L_span T_st v_des Hip_Height delta 
t=pha_sw;

x0=-L_span;
xm=0;
xd0=-v_des;
xdm=0;
y0=Hip_Height;
ym=-0.426;
yd0=delta*pi/T_st;
ydm=0;

key_point_x=[x0;xm;xd0;xdm;0;0];
key_point_y=[y0;ym;yd0;ydm;0;0];

tf=0.5;

 X = [
    0           0           0         0       0   1
    tf^5        tf^4        tf^3      tf^2    tf  1
    0           0           0         0       1   0
    5*tf^4      4*tf^3      3*tf^2    2*tf    1   0
    0           0           0         2       0   0
    20*tf^3     12*tf^2     6*tf      2       0   0
 ];
y_coeffs = (X \ key_point_y)';%左除相当于inv(X)*key_point_y
x_coeffs = (X \ key_point_x)';

% coefficients of derivatives 
x_coeffs_d = x_coeffs(1:5) .* (5:-1:1);
y_coeffs_d = y_coeffs(1:5) .* (5:-1:1);
% coeffs_dd = coeffs_d(1:4) .* (4:-1:1);
 
if pha_sw<=0.5
    px=polyval(x_coeffs,t); 
    py=polyval(y_coeffs,t);
    P_des=[px;py];
    pdx = polyval(x_coeffs_d, t);
    pdy = polyval(y_coeffs_d, t);
    Pd_des=[pdx;pdy];
else
    px=-polyval(x_coeffs,1-t); 
    py=polyval(y_coeffs,1-t);
    P_des=[px;py];
    pdx = polyval(x_coeffs_d, 1-t);
    pdy = -polyval(y_coeffs_d, 1-t);
    Pd_des=[pdx;pdy];
end

