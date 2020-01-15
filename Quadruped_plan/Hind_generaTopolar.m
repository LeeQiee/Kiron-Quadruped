function [P_polar,Pd_polar,Jpolar]=Hind_generaTopolar(Q,Qd)
%----给定两条后腿广义关节速度、加速度，计算出相应的极坐标位置和速度以及雅可比矩阵----%
global L1 L2
% LF_q0=LF_Q(1);
q1=Q(2);q2=Q(3);
% RF_q0=RF_Q(1);
% RF_q1=RF_Q(2);RF_q2=RF_Q(3);

% LF_qd0=LF_Qd(1);
qd1=Qd(2);qd2=Qd(3);
% RF_qd0=RF_Qd(1);
% RF_qd1=RF_Qd(2);RF_qd2=RF_Qd(3);


r=sqrt(L1^2+L2^2-2*L1*L2*cos(pi-q2));
theta=q1+asin(L2*sin(pi-q2)/r);
P_polar=[r;theta];

% RF_r=sqrt(L1^2+L2^2-2*L1*L2*cos(pi+RF_q2));
% RF_theta=RF_q1-asin(L2*sin(pi+RF_q2)/RF_r);
% RF_P_polar=[RF_r;RF_theta];

J11=0;J12=-(27*sin(q2))/(250*((27*cos(q2))/125 + 549/2500)^(1/2));
J21=1;J22=((3*cos(q2))/(10*((27*cos(q2))/125 + 549/2500)^(1/2)) + (81*sin(q2)^2)/(2500*((27*cos(q2))/125 + 549/2500)^(3/2)))/(1 - (9*sin(q2)^2)/((108*cos(q2))/5 + 549/25))^(1/2);
Jpolar=[J11,J12;J21,J22];
Pd_polar=Jpolar*[qd1;qd2];

