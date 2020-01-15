function [P_car_des,Pd_car_des]=Quad_Stance_traj(pha_st)
%-----给定相位信号，计算笛卡尔坐标系下足端规划轨迹位置和速度-----%
global L_span T_st Hip_Height delta 

px_des=L_span*(1-2*pha_st);
pz_des=-delta*cos(pi*(1-2*pha_st)/2)+Hip_Height;
P_car_des=[px_des;pz_des];

pxd_des=-2*L_span/T_st;
pzd_des=-(delta*pi/T_st)*sin(pi*(1-2*pha_st)/2);
Pd_car_des=[pxd_des;pzd_des];
