function [P_des,Pd_des]=Stance_tra(pha_st)
%-----给定相位信号，计算笛卡尔坐标系下足端规划轨迹位置和速度-----%
global L_span T_st Hip_Height delta 

px_des=L_span*(1-2*pha_st);
py_des=delta*cos(pi*px_des/(2*L_span))+Hip_Height;
P_des=[px_des;py_des];

pxd_des=-2*L_span/T_st;
pyd_des=(delta*pi/T_st)*sin(pi*px_des/(2*L_span));
Pd_des=[pxd_des;pyd_des];
