function [P_polar,Pd_polar,J_polar]=Forward_Kine(q,qd,PitchAngle)
%----正运动学，给定关节角度、速度，计算极坐标下位置和速度----%
global L_leg

P_polar=[2*L_leg*cos(q(2,1)/2);q(1,1)+PitchAngle+q(2,1)/2];
J_polar=[0,-L_leg*sin(q(2,1)/2);1,0.5];
Pd_polar=J_polar*qd;
