function y=trot_insitu_traj(u)
%----给定时间信号和接触信号，计算极坐标系下的足端轨迹规划-----%
global T_st T_sw T_strid
tim=u;
% in_ctact=double(u(2));
if tim<1.5
    [LF_P_car_des,LF_Pd_car_des]=Quad_Stance_traj(0.5);
    [RF_P_car_des,RF_Pd_car_des]=Quad_Stance_traj(0.5);
    [LF_P_polar_des,LF_Pd_polar_des]=carTopolar(LF_P_car_des,LF_Pd_car_des);
    [RF_P_polar_des,RF_Pd_polar_des]=carTopolar(RF_P_car_des,RF_Pd_car_des);
else
    pha=mod((tim-1.5),T_strid);
    if pha<=T_st
        LF_pha_st=pha/T_st;
        RF_pha_sw=LF_pha_st;
        [LF_P_car_des,LF_Pd_car_des]=Quad_Stance_traj(LF_pha_st);
        [RF_P_car_des,RF_Pd_car_des]=Quad_Swing_traj(RF_pha_sw);
        [LF_P_polar_des,LF_Pd_polar_des]=carTopolar(LF_P_car_des,LF_Pd_car_des);
        [RF_P_polar_des,RF_Pd_polar_des]=carTopolar(RF_P_car_des,RF_Pd_car_des);
    else
        LF_pha_sw=(pha-T_st)/T_sw;
        RF_pha_st=LF_pha_sw;
        [LF_P_car_des,LF_Pd_car_des]=Quad_Swing_traj(LF_pha_sw);
        [RF_P_car_des,RF_Pd_car_des]=Quad_Stance_traj(RF_pha_st);
        [LF_P_polar_des,LF_Pd_polar_des]=carTopolar(LF_P_car_des,LF_Pd_car_des);
        [RF_P_polar_des,RF_Pd_polar_des]=carTopolar(RF_P_car_des,RF_Pd_car_des);
    end
end
y=[LF_P_polar_des;RF_P_polar_des;LF_Pd_polar_des;RF_Pd_polar_des];

