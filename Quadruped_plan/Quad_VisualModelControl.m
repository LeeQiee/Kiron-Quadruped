function tau=Quad_VisualModelControl(u)
%----虚拟模型控制，给定关节变量，计算驱动力矩----%
global Kp_r Kd_r Kp_theta Kd_theta Kp_hip Kd_hip
LF_Q=u(1:3);RF_Q=u(4:6);RH_Q=u(7:9);LH_Q=u(10:12);

LF_Qd=u(13:15);RF_Qd=u(16:18);RH_Qd=u(19:21);LH_Qd=u(22:24);

LF_q0=u(1);RF_q0=u(4);RH_q0=u(7);LH_q0=u(10);
LF_qd0=u(13);RF_qd0=u(16);RH_qd0=u(19);LH_qd0=u(22);

LF_P_polar_des=u(25:26);RH_P_polar_des=u(33:34);
RF_P_polar_des=u(27:28);LH_P_polar_des=u(35:36);
LF_Pd_polar_des=u(29:30);RH_Pd_polar_des=u(37:38);
RF_Pd_polar_des=u(31:32);LH_Pd_polar_des=u(39:40);
LF_Q0_des=u(41:42);RF_Q0_des=u(43:44);RH_Q0_des=u(45:46);LH_Q0_des=u(47:48);
%左前腿
[LF_P_polar,LF_Pd_polar,LF_Jpolar]=Front_generaTopolar(LF_Q,LF_Qd);
LF_e_polar=LF_P_polar_des - LF_P_polar;
LF_ed_polar=LF_Pd_polar_des - LF_Pd_polar;
LF_Fr=Kp_r*LF_e_polar(1) + Kd_r*LF_ed_polar(1);
LF_Ftheta=Kp_theta*LF_e_polar(2) + Kd_theta*LF_ed_polar(2);
LF_tau12=LF_Jpolar'*[LF_Fr;LF_Ftheta];
LF_tau0=Kp_hip*(LF_Q0_des(1)-LF_q0)+Kd_hip*(LF_Q0_des(2)-LF_qd0);
LF_tau=[LF_tau0;LF_tau12];
%右前腿
[RF_P_polar,RF_Pd_polar,RF_Jpolar]=Front_generaTopolar(RF_Q,RF_Qd);
RF_e_polar=RF_P_polar_des - RF_P_polar;
RF_ed_polar=RF_Pd_polar_des - RF_Pd_polar;
RF_Fr=Kp_r*RF_e_polar(1) + Kd_r*RF_ed_polar(1);
RF_Ftheta=Kp_theta*RF_e_polar(2) + Kd_theta*RF_ed_polar(2);
RF_tau12=RF_Jpolar'*[RF_Fr;RF_Ftheta];
RF_tau0=Kp_hip*(RF_Q0_des(1)-RF_q0)+Kd_hip*(RF_Q0_des(2)-RF_qd0);
RF_tau=[RF_tau0;RF_tau12];
%右后腿
[RH_P_polar,RH_Pd_polar,RH_Jpolar]=Front_generaTopolar(RH_Q,RH_Qd);
RH_e_polar=RH_P_polar_des - RH_P_polar;
RH_ed_polar=RH_Pd_polar_des - RH_Pd_polar;
RH_Fr=Kp_r*RH_e_polar(1) + Kd_r*RH_ed_polar(1);
RH_Ftheta=Kp_theta*RH_e_polar(2) + Kd_theta*RH_ed_polar(2);
RH_tau12=RH_Jpolar'*[RH_Fr;RH_Ftheta];
RH_tau0=Kp_hip*(RH_Q0_des(1)-RH_q0)+Kd_hip*(RH_Q0_des(2)-RH_qd0);
RH_tau=[RH_tau0;RH_tau12];
%左后腿
[LH_P_polar,LH_Pd_polar,LH_Jpolar]=Front_generaTopolar(LH_Q,LH_Qd);
LH_e_polar=LH_P_polar_des - LH_P_polar;
LH_ed_polar=LH_Pd_polar_des - LH_Pd_polar;
LH_Fr=Kp_r*LH_e_polar(1) + Kd_r*LH_ed_polar(1);
LH_Ftheta=Kp_theta*LH_e_polar(2) + Kd_theta*LH_ed_polar(2);
LH_tau12=LH_Jpolar'*[LH_Fr;LH_Ftheta];
LH_tau0=Kp_hip*(LH_Q0_des(1)-LH_q0)+Kd_hip*(LH_Q0_des(2)-LH_qd0);
LH_tau=[LH_tau0;LH_tau12];

% tau=[LF_tau;RF_tau;RH_tau;LH_tau];
tau=[LF_tau;RF_tau;RH_tau;LH_tau];

