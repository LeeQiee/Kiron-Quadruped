function y=walk_traj(u)
%----给定时间信号和接触信号，计算极坐标系下的足端轨迹规划-----%
global  T_strid Hip_Height walk_duty
persistent Q0_des
tim=u(1);
pitch=u(2);roll=u(3);roll_d=u(4);
% adjust_pitch=0*0.85*tan(pitch);
% adjust_roll=0*0.35*tan(roll);
% adjust_LF_Pz=-adjust_pitch+adjust_roll;
% adjust_RH_Pz=adjust_pitch-adjust_roll;
% adjust_RF_Pz=-adjust_pitch-adjust_roll;
% adjust_LH_Pz=adjust_pitch+adjust_roll;
% if adjust_Pz<-0.15 %限幅
%     adjust_Pz=-0.15;
% end
if tim<=1.5        %整体机身稳定
    LF_P_car_des=[0;Hip_Height]; 
    LF_Pd_car_des=[0;0];
    [LF_P_polar_des,LF_Pd_polar_des]=carTopolar(LF_P_car_des,LF_Pd_car_des);
    RF_P_polar_des=LF_P_polar_des;RF_Pd_polar_des=LF_Pd_polar_des;
    RH_P_polar_des=LF_P_polar_des;RH_Pd_polar_des=LF_Pd_polar_des;
    LH_P_polar_des=LF_P_polar_des;LH_Pd_polar_des=LF_Pd_polar_des;
    Q0_des=[0;0;0;0;0;0;0;0];
elseif tim>1.5 && tim<2     %初始阶段，周期运动前的准备动作
    LF_pha_sw=(tim-1.5)/0.5;
    [LF_P_car_des,LF_Pd_car_des]=Quad_Swing_traj_init(LF_pha_sw);          
    [LF_P_polar_des,LF_Pd_polar_des]=carTopolar(LF_P_car_des,LF_Pd_car_des);
    
    RF_P_car_des=[0;Hip_Height];  RF_Pd_car_des=[0;0];
    [RF_P_polar_des,RF_Pd_polar_des]=carTopolar(RF_P_car_des,RF_Pd_car_des);
    LH_P_polar_des=RF_P_polar_des;LH_Pd_polar_des=RF_Pd_polar_des;
    RH_P_polar_des=RF_P_polar_des;RH_Pd_polar_des=RF_Pd_polar_des;
    Q0_des=[0;0;0;0;0;0;0;0];
elseif tim>=2          %周期运动开始    
    t_residue=mod(2,T_strid);
    t_strid=mod((tim-t_residue+0.5*(1+walk_duty)*T_strid),T_strid);%以左前腿的相位为参考
    pha=t_strid/T_strid;
    
    if pha>=0 && pha<walk_duty-0.75
        LF_pha_st=pha/walk_duty;RH_pha_st=(pha+0.75)/walk_duty;RF_pha_st=(pha+0.5)/walk_duty;LH_pha_st=(pha+0.25)/walk_duty;
        [LF_P_car_des,LF_Pd_car_des]=Quad_Stance_traj(LF_pha_st);[RH_P_car_des,RH_Pd_car_des]=Quad_Stance_traj(RH_pha_st);
        [RF_P_car_des,RF_Pd_car_des]=Quad_Stance_traj(RF_pha_st);[LH_P_car_des,LH_Pd_car_des]=Quad_Stance_traj(LH_pha_st);
    elseif pha>=walk_duty-0.75 && pha<0.25
        LF_pha_st=pha/walk_duty;RH_pha_sw=(pha-0.05)/(1-walk_duty);RF_pha_st=(pha+0.5)/walk_duty;LH_pha_st=(pha+0.25)/walk_duty;
        [LF_P_car_des,LF_Pd_car_des]=Quad_Stance_traj(LF_pha_st);[RH_P_car_des,RH_Pd_car_des]=Quad_Swing_traj(RH_pha_sw);
        [RF_P_car_des,RF_Pd_car_des]=Quad_Stance_traj(RF_pha_st);[LH_P_car_des,LH_Pd_car_des]=Quad_Stance_traj(LH_pha_st);
    elseif pha>=0.25 && pha<walk_duty-0.5
        LF_pha_st=pha/walk_duty;RH_pha_st=(pha-0.25)/walk_duty;RF_pha_st=(pha+0.5)/walk_duty;LH_pha_st=(pha+0.25)/walk_duty;
        [LF_P_car_des,LF_Pd_car_des]=Quad_Stance_traj(LF_pha_st);[RH_P_car_des,RH_Pd_car_des]=Quad_Stance_traj(RH_pha_st);
        [RF_P_car_des,RF_Pd_car_des]=Quad_Stance_traj(RF_pha_st);[LH_P_car_des,LH_Pd_car_des]=Quad_Stance_traj(LH_pha_st);
    elseif pha>=walk_duty-0.5 && pha<0.5
        LF_pha_st=pha/walk_duty;RH_pha_st=(pha-0.25)/walk_duty;RF_pha_sw=(pha-walk_duty+0.5)/(1-walk_duty);LH_pha_st=(pha+0.25)/walk_duty;
        [LF_P_car_des,LF_Pd_car_des]=Quad_Stance_traj(LF_pha_st);[RH_P_car_des,RH_Pd_car_des]=Quad_Stance_traj(RH_pha_st);
        [RF_P_car_des,RF_Pd_car_des]=Quad_Swing_traj(RF_pha_sw);[LH_P_car_des,LH_Pd_car_des]=Quad_Stance_traj(LH_pha_st);
    elseif pha>=0.5 && pha<walk_duty-0.25
        LF_pha_st=pha/walk_duty;RH_pha_st=(pha-0.25)/walk_duty;RF_pha_st=(pha-0.5)/walk_duty;LH_pha_st=(pha+0.25)/walk_duty;
        [LF_P_car_des,LF_Pd_car_des]=Quad_Stance_traj(LF_pha_st);[RH_P_car_des,RH_Pd_car_des]=Quad_Stance_traj(RH_pha_st);
        [RF_P_car_des,RF_Pd_car_des]=Quad_Stance_traj(RF_pha_st);[LH_P_car_des,LH_Pd_car_des]=Quad_Stance_traj(LH_pha_st);
    elseif pha>=walk_duty-0.25 && pha<0.75
        LF_pha_st=pha/walk_duty;RH_pha_st=(pha-0.25)/walk_duty;RF_pha_st=(pha-0.5)/walk_duty;LH_pha_sw=(pha-walk_duty+0.25)/(1-walk_duty);
        [LF_P_car_des,LF_Pd_car_des]=Quad_Stance_traj(LF_pha_st);[RH_P_car_des,RH_Pd_car_des]=Quad_Stance_traj(RH_pha_st);
        [RF_P_car_des,RF_Pd_car_des]=Quad_Stance_traj(RF_pha_st);[LH_P_car_des,LH_Pd_car_des]=Quad_Swing_traj(LH_pha_sw);
    elseif pha>=0.75 && pha<walk_duty
        LF_pha_st=pha/walk_duty;RH_pha_st=(pha-0.25)/walk_duty;RF_pha_st=(pha-0.5)/walk_duty;LH_pha_st=(pha-0.75)/walk_duty;
        [LF_P_car_des,LF_Pd_car_des]=Quad_Stance_traj(LF_pha_st);[RH_P_car_des,RH_Pd_car_des]=Quad_Stance_traj(RH_pha_st);
        [RF_P_car_des,RF_Pd_car_des]=Quad_Stance_traj(RF_pha_st);[LH_P_car_des,LH_Pd_car_des]=Quad_Stance_traj(LH_pha_st);
    elseif pha>=walk_duty && pha<=1
        LF_pha_sw=(pha-walk_duty)/(1-walk_duty);RH_pha_st=(pha-0.25)/walk_duty;RF_pha_st=(pha-0.5)/walk_duty;LH_pha_st=(pha-0.75)/walk_duty;
        [LF_P_car_des,LF_Pd_car_des]=Quad_Swing_traj(LF_pha_sw);[RH_P_car_des,RH_Pd_car_des]=Quad_Stance_traj(RH_pha_st);
        [RF_P_car_des,RF_Pd_car_des]=Quad_Stance_traj(RF_pha_st);[LH_P_car_des,LH_Pd_car_des]=Quad_Stance_traj(LH_pha_st);
    end
    [LF_P_polar_des,LF_Pd_polar_des]=carTopolar(LF_P_car_des,LF_Pd_car_des);
    [RH_P_polar_des,RH_Pd_polar_des]=carTopolar(RH_P_car_des,RH_Pd_car_des);
    [RF_P_polar_des,RF_Pd_polar_des]=carTopolar(RF_P_car_des,RF_Pd_car_des);        
    [LH_P_polar_des,LH_Pd_polar_des]=carTopolar(LH_P_car_des,LH_Pd_car_des);
    Q0_des=[0;0;0;0;0;0;0;0];
%         %----上坡姿态调整-----%
%         LF_P_car_des=LF_P_car_des+[0;adjust_LF_Pz];
%         RH_P_car_des=RH_P_car_des+[0;adjust_RH_Pz];
%         RF_P_car_des=RF_P_car_des+[0;adjust_RF_Pz];
%         LH_P_car_des=LH_P_car_des+[0;adjust_LH_Pz];
end
y=[LF_P_polar_des;RF_P_polar_des;LF_Pd_polar_des;RF_Pd_polar_des;...
   RH_P_polar_des;LH_P_polar_des;RH_Pd_polar_des;LH_Pd_polar_des;Q0_des];