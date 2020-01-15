function tau=VisualModelControl(u)
%----虚拟模型控制，给定关节变量，计算驱动力矩----%
global Kp_r Kd_r Kp_theta Kd_theta

x=u(1:13);
q=u(14:15);
qd=u(16:17);
P_polar_des=u(18:19);
Pd_polar_des=u(20:21);

% Q=fbanim(x,q);
% beta=Q(4);
beta=0;
% q=q+[-0.2269;0.902];
[P_polar,Pd_polar,J_polar]=Forward_Kine(q,qd,beta);
e_polar=P_polar_des-P_polar;
ed_polar=Pd_polar_des-Pd_polar;

Fr=Kp_r*e_polar(1,1)+Kd_r*ed_polar(1,1);
Ftheta=Kp_theta*e_polar(2,1)+Kd_theta*ed_polar(2,1);
tau=J_polar'*[Fr;Ftheta];


