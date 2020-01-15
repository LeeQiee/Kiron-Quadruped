function [P_des,Pd_des]=Swing_tra_init(pha_sw_init)
%-----给定相位信号，计算摆动相足端规划轨迹-----%
global L_span T_st v_des Hip_Height delta 
t=pha_sw_init;

xf=L_span;
xm=0;
xdf=-v_des;
xdm=0;
yf=Hip_Height;
ym=1;
ydf=delta*pi/T_st;
ydm=0;

key_point_x=[xm;xf;xdm;xdf;0;0];
key_point_y=[ym;yf;ydm;ydf;0;0];

t0=0.5;
tf=1;

 X = [
    t0^5        t0^4        t0^3      t0^2    t0  1
    tf^5        tf^4        tf^3      tf^2    tf  1
    5*t0^4      4*t0^3      3*t0^2    2*t0    1   0
    5*tf^4      4*tf^3      3*tf^2    2*tf    1   0
    20*t0^3     12*t0^2     6*t0      2       0   0
    20*tf^3     12*tf^2     6*tf      2       0   0
 ];
y_coeffs = (X \ key_point_y)';%左除相当于inv(X)*key_point_y
x_coeffs = (X \ key_point_x)';
x_coeffs_d = x_coeffs(1:5) .* (5:-1:1);
y_coeffs_d = y_coeffs(1:5) .* (5:-1:1);

px=polyval(x_coeffs,t); 
py=polyval(y_coeffs,t);
pdx=polyval(x_coeffs_d,t); 
pdy=polyval(y_coeffs_d,t);

P_des=[px;py];
Pd_des=[pdx;pdy];
