function Qdd_des=Quad_Qdd_Compute(u)
%----逆动力学控制，给定关节变量，计算期望广义加速度----%
global ID_Kp ID_Kd
LF_Q=u(2:3);RF_Q=u(5:6);RH_Q=u(8:9);LH_Q=u(11:12);

LF_Qd=u(14:15);RF_Qd=u(17:18);RH_Qd=u(20:21);LH_Qd=u(23:24);

LF_Q_des=u(25:26);RH_Q_des=LF_Q_des;
LF_Qd_des=u(27:28);RH_Qd_des=LF_Qd_des;
RF_Q_des=u(29:30);LH_Q_des=RF_Q_des;
RF_Qd_des=u(31:32);LH_Qd_des=RF_Qd_des;

LF_Qdd_des=ID_Kp*(LF_Q_des-LF_Q)+ID_Kd*(LF_Qd_des-LF_Qd);
RF_Qdd_des=ID_Kp*(RF_Q_des-RF_Q)+ID_Kd*(RF_Qd_des-RF_Qd);
RH_Qdd_des=ID_Kp*(RH_Q_des-RH_Q)+ID_Kd*(RH_Qd_des-RH_Qd);
LH_Qdd_des=ID_Kp*(LH_Q_des-LH_Q)+ID_Kd*(LH_Qd_des-LH_Qd);

Qdd_des=[0;LF_Qdd_des;0;RF_Qdd_des;0;RH_Qdd_des;0;LH_Qdd_des];
