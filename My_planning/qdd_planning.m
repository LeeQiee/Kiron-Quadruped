function  qdd_des= qdd_planning(u)
%----广义关节加速度qdd规划-----%
  x = u(1:13);
  xd = u(14:26);
  q = u(27:28);
  qd = u(29:30);
  in_ctact =double(u(31));
  
  persistent ctact_ahead_now
  if isempty(ctact_ahead_now)
      ctact_ahead_now=[0 0];
  end
  %----每次调用函数更新接触判断-----%
  ctact_ahead_now(1)=ctact_ahead_now(2);
  ctact_ahead_now(2)=in_ctact;
  
  body_vy=xd(12);%反馈的前进速度
  Q=fbanim(x,q);
  beta=Q(4);
  T_st=0.2;%站立相时间
  vy_des=2;%期望的前进速度
  Kv=1;%速度增益,Raibert三段法控前进速度
  
  qdd_des(1,1)=0;
  qdd_des(2,1)=20*(2*acos(0.99)-q(2,1))+5*(0-qd(2,1));

%   if in_ctact==0
%      Kq_sw=10;%关节位置增益
%      Kqd_sw=1;%关节速度增益
%      if ctact_ahead_now(1)==1 %表明是从站立相到摆动相的过渡时刻，更新摆动相初始值
%          q_sw_init=q;
%          qd_sw_init=qd;
%      end
%      bias_RelBody=0.5*body_vy*T_st + Kv*(body_vy-vy_des);
%      theta=asin(bias_RelBody/0.9);
%      q_des(1,1)=(0.5*pi-(theta-beta))-acos(0.9);
%      q_des(2,1)=2*acos(0.9);
%      qd_des=[0;0];
%      qdd_des=Kq_sw*(q_des-q)+Kqd_sw*(qd_des-qd);
%   else
%      Kq_st=10;
%      Kqd_st=1;
%      if ctact_ahead_now(1)==0 %表明是从摆动相到站立相的过渡时刻，更新站立相初始值
%         q_st_init=q;
%         qd_st_init=qd;
%         beta_st_init=beta;         
%      end
%      qdd_des(1,1)=Kq_st*(0-beta)+Kqd_st*(-x(8));
%      qdd_des(2,1)=20*(2*acos(0.9)-q(2,1))+5*(0-qd(2,1));
%   end
%   