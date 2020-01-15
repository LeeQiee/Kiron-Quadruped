function y=LegFoot_traj(u)
%----给定时间信号和接触信号，计算极坐标系下的足端轨迹规划-----%
global T_st T_sw 
tim=u(1);
in_ctact=double(u(2));

persistent tim_ref
if isempty(tim_ref)
    tim_ref=0;
end

persistent ctact_ahead_now
if isempty(ctact_ahead_now)
    ctact_ahead_now=[0 0];
end
%----每次调用函数更新接触判断-----%
ctact_ahead_now(1)=ctact_ahead_now(2);
ctact_ahead_now(2)=in_ctact;

if (ctact_ahead_now(1)==0)&&(ctact_ahead_now(2)==1)%触地时刻
    tim_ref=tim;
    [P_des,Pd_des]=Stance_tra(0);
    [P_polar_des,Pd_polar_des]=polar_transform(P_des,Pd_des);
end
if (ctact_ahead_now(1)==1)&&(ctact_ahead_now(2)==1)%站立相
    pha_st=(tim-tim_ref)/T_st;
    [P_des,Pd_des]=Stance_tra(pha_st);
    [P_polar_des,Pd_polar_des]=polar_transform(P_des,Pd_des);
end
if (ctact_ahead_now(1)==1)&&(ctact_ahead_now(2)==0)%离地时刻
    [P_des,Pd_des]=Swing_tra(0);
    [P_polar_des,Pd_polar_des]=polar_transform(P_des,Pd_des);
end
if (ctact_ahead_now(1)==0)&&(ctact_ahead_now(2)==0)%摆动相
    if tim_ref<T_sw*0.5
        pha_sw_init=(tim+T_sw/2)/T_sw;
        [P_des,Pd_des]=Swing_tra_init(pha_sw_init);
        [P_polar_des,Pd_polar_des]=polar_transform(P_des,Pd_des);
    else
        pha_sw=(tim-tim_ref-T_st)/T_sw;
        [P_des,Pd_des]=Swing_tra(pha_sw);
        [P_polar_des,Pd_polar_des]=polar_transform(P_des,Pd_des);
    end
end
% y=[P_des;Pd_des];
y=[P_polar_des;Pd_polar_des];