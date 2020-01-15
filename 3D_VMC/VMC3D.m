function y=VMC3D(u)

Q=u(1:12);Qd=u(13:24);
ROLL=u(25:26);
PITCH=u(27:28);
YAW=u(29:30);
sw_init_para=u(31:51);
st_para=u(52:61);
state_now=u(62);
Hip_V=u(63:74);

LF_Q=Q(1:3);RF_Q=Q(4:6);RH_Q=Q(7:9);LH_Q=Q(10:12);
% LF_Qd=u(13:15);RF_Qd=u(16:18);RH_Qd=u(19:21);LH_Qd=u(22:24);
LF_Qd=Qd(1:3);RF_Qd=Qd(4:6);RH_Qd=Qd(7:9);LH_Qd=Qd(10:12);

LF_sw_initPfoot=sw_init_para(1:3);   RF_sw_initPfoot=sw_init_para(4:6);
RH_sw_initPfoot=sw_init_para(7:9);   LH_sw_initPfoot=sw_init_para(10:12);
LF_sw_initHip_v=sw_init_para(13:14); RF_sw_initHip_v=sw_init_para(15:16);
RH_sw_initHip_v=sw_init_para(17:18); LH_sw_initHip_v=sw_init_para(19:20);
pha_sw=sw_init_para(21);

st_Pd_des=st_para(2:4);
initHip_V_des=st_para(2:3);
ROLL_des=st_para(5:6);
pitch_des=st_para(7);pitchd_des=st_para(8);
YAW_des=st_para(9:10);

LF_Hip_V=Hip_V(1:3);RF_Hip_V=Hip_V(4:6);RH_Hip_V=Hip_V(7:9);LH_Hip_V=Hip_V(10:12);

[LF_st_P_des,RF_st_P_des,RH_st_P_des,LH_st_P_des]=CalHipHeight(-st_para(1),ROLL,PITCH);
[delta_torque,delta_fx]=CalDelta_fy_torque(YAW,YAW_des,ROLL,ROLL_des);

if     state_now==4 %初始阶段，四足站立
    [LF_tau,LF_F]= CalLegTorque(LF_st_P_des,st_Pd_des,LF_Q,LF_Qd,-delta_fx,LF_Hip_V) ;
    [RF_tau,RF_F]= CalLegTorque(RF_st_P_des,st_Pd_des,RF_Q,RF_Qd, delta_fx,RF_Hip_V) ;
    [RH_tau,RH_F]= CalLegTorque(RH_st_P_des,st_Pd_des,RH_Q,RH_Qd, delta_fx,RH_Hip_V) ;
    [LH_tau,LH_F]= CalLegTorque(LH_st_P_des,st_Pd_des,LH_Q,LH_Qd,-delta_fx,LH_Hip_V) ;
    LF_tau=-LF_tau+[-delta_torque;0;0];
    RF_tau=-RF_tau+[-delta_torque;0;0];
    RH_tau=-RH_tau+[-delta_torque;0;0];
    LH_tau=-LH_tau+[-delta_torque;0;0];
elseif state_now==0 || state_now==3 %LF与RH腿摆动，RF与LH支撑
    [LF_sw_P_des,LF_sw_Pd_des]=cycloid_curve_swing(pha_sw,LF_sw_initPfoot,LF_sw_initHip_v,initHip_V_des);
    [LF_tau,LF_F]=CalLegTorque(LF_sw_P_des,LF_sw_Pd_des,LF_Q,LF_Qd);
    [RH_sw_P_des,RH_sw_Pd_des]=cycloid_curve_swing(pha_sw,RH_sw_initPfoot,RH_sw_initHip_v,initHip_V_des);
    [RH_tau,RH_F]=CalLegTorque(RH_sw_P_des,RH_sw_Pd_des,RH_Q,RH_Qd);
    [RF_tau,RF_F]= CalLegTorque(RF_st_P_des,st_Pd_des,RF_Q,RF_Qd, delta_fx,RF_Hip_V) ;
    [LH_tau,LH_F]= CalLegTorque(LH_st_P_des,st_Pd_des,LH_Q,LH_Qd,-delta_fx,LH_Hip_V) ;
    RF_tau=-RF_tau+[-delta_torque;0;0];
    LH_tau=-LH_tau+[-delta_torque;0;0];
elseif state_now==1.1 %LF先进入支撑，RH继续摆动，RF与LH支撑
    [LF_tau,LF_F]= CalLegTorque(LF_st_P_des,st_Pd_des,LF_Q,LF_Qd,-delta_fx,LF_Hip_V) ;
    [RH_sw_P_des,RH_sw_Pd_des]=cycloid_curve_swing(pha_sw,RH_sw_initPfoot,RH_sw_initHip_v,initHip_V_des);
    [RH_tau,RH_F]=CalLegTorque(RH_sw_P_des,RH_sw_Pd_des,RH_Q,RH_Qd);
    [RF_tau,RF_F]= CalLegTorque(RF_st_P_des,st_Pd_des,RF_Q,RF_Qd, delta_fx,RF_Hip_V) ;
    [LH_tau,LH_F]= CalLegTorque(LH_st_P_des,st_Pd_des,LH_Q,LH_Qd,-delta_fx,LH_Hip_V) ; 
    LF_tau=-LF_tau+[-delta_torque;0;0];
    RF_tau=-RF_tau+[-delta_torque;0;0];
    LH_tau=-LH_tau+[-delta_torque;0;0];
elseif state_now==1.2 %LF继续摆动，RH先进入支撑，RF与LH支撑
    [LF_sw_P_des,LF_sw_Pd_des]=cycloid_curve_swing(pha_sw,LF_sw_initPfoot,LF_sw_initHip_v,initHip_V_des);
    [LF_tau,LF_F]=CalLegTorque(LF_sw_P_des,LF_sw_Pd_des,LF_Q,LF_Qd);
    [RH_tau,RH_F]= CalLegTorque(RH_st_P_des,st_Pd_des,RH_Q,RH_Qd, delta_fx,RH_Hip_V) ;
    [RF_tau,RF_F]= CalLegTorque(RF_st_P_des,st_Pd_des,RF_Q,RF_Qd, delta_fx,RF_Hip_V) ;
    [LH_tau,LH_F]= CalLegTorque(LH_st_P_des,st_Pd_des,LH_Q,LH_Qd,-delta_fx,LH_Hip_V) ; 
    RF_tau=-RF_tau+[-delta_torque;0;0];
    RH_tau=-RH_tau+[-delta_torque;0;0];
    LH_tau=-LH_tau+[-delta_torque;0;0];
elseif state_now==2 || state_now==1 %LF与RH腿支撑，RF与LH摆动
    [LF_tau,LF_F]= CalLegTorque(LF_st_P_des,st_Pd_des,LF_Q,LF_Qd,-delta_fx,LF_Hip_V) ;
    [RH_tau,RH_F]= CalLegTorque(RH_st_P_des,st_Pd_des,RH_Q,RH_Qd, delta_fx,RH_Hip_V) ;
    LF_tau=-LF_tau+[-delta_torque;0;0];
    RH_tau=-RH_tau+[-delta_torque;0;0];
    [RF_sw_P_des,RF_sw_Pd_des]=cycloid_curve_swing(pha_sw,RF_sw_initPfoot,RF_sw_initHip_v,initHip_V_des);
    [RF_tau,RF_F]=CalLegTorque(RF_sw_P_des,RF_sw_Pd_des,RF_Q,RF_Qd);
    [LH_sw_P_des,LH_sw_Pd_des]=cycloid_curve_swing(pha_sw,LH_sw_initPfoot,LH_sw_initHip_v,initHip_V_des);
    [LH_tau,LH_F]=CalLegTorque(LH_sw_P_des,LH_sw_Pd_des,LH_Q,LH_Qd);    
elseif state_now==3.1 %LF与RH支撑，RF先进入支撑，LH继续摆动
    [LF_tau,LF_F]= CalLegTorque(LF_st_P_des,st_Pd_des,LF_Q,LF_Qd,-delta_fx,LF_Hip_V) ;
    [RH_tau,RH_F]= CalLegTorque(RH_st_P_des,st_Pd_des,RH_Q,RH_Qd, delta_fx,RH_Hip_V) ;
    [RF_tau,RF_F]= CalLegTorque(RF_st_P_des,st_Pd_des,RF_Q,RF_Qd, delta_fx,RF_Hip_V) ;
    LF_tau=-LF_tau+[-delta_torque;0;0];
    RH_tau=-RH_tau+[-delta_torque;0;0];
    RF_tau=-RF_tau+[-delta_torque;0;0];
    [LH_sw_P_des,LH_sw_Pd_des]=cycloid_curve_swing(pha_sw,LH_sw_initPfoot,LH_sw_initHip_v,initHip_V_des);
    [LH_tau,LH_F]=CalLegTorque(LH_sw_P_des,LH_sw_Pd_des,LH_Q,LH_Qd);
elseif state_now==3.2 %LF与RH支撑，LH先进入支撑，RF继续摆动
    [LF_tau,LF_F]= CalLegTorque(LF_st_P_des,st_Pd_des,LF_Q,LF_Qd,-delta_fx,LF_Hip_V) ;
    [RH_tau,RH_F]= CalLegTorque(RH_st_P_des,st_Pd_des,RH_Q,RH_Qd, delta_fx,RH_Hip_V) ;
    [LH_tau,LH_F]= CalLegTorque(LH_st_P_des,st_Pd_des,LH_Q,LH_Qd,-delta_fx,LH_Hip_V) ; 
    LF_tau=-LF_tau+[-delta_torque;0;0];
    RH_tau=-RH_tau+[-delta_torque;0;0];
    LH_tau=-LH_tau+[-delta_torque;0;0];
    [RF_sw_P_des,RF_sw_Pd_des]=cycloid_curve_swing(pha_sw,RF_sw_initPfoot,RF_sw_initHip_v,initHip_V_des);
    [RF_tau,RF_F]=CalLegTorque(RF_sw_P_des,RF_sw_Pd_des,RF_Q,RF_Qd);
end
Tau=[LF_tau;RF_tau;RH_tau;LH_tau];
Visual_F=[LF_F;RF_F;RH_F;LH_F];   
y=[Tau;Visual_F];