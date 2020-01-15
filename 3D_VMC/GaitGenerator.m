function y=GaitGenerator(u)

global Hip_Height vx_des vy_des T_st

persistent tim_sw_start LF_init_Hip_Vxy RF_init_Hip_Vxy RH_init_Hip_Vxy LH_init_Hip_Vxy...
           LF_LiftOff_P RF_LiftOff_P RH_LiftOff_P LH_LiftOff_P
% if isempty(init_Hip_vx)
%    init_Hip_vx=0;
% end
% if isempty(init_Hip_vy)
%    init_Hip_vy=0;
% end
% if isempty(tim_sw_start)
%    tim_sw_start=0;
% end
% if isempty(Quad_LiftOff_P)
%    Quad_LiftOff_P=[0;0;Hip_Height;0;0;Hip_Height;0;0;Hip_Height;0;0;Hip_Height];
% end
Q=u(1:12);tim=u(13);S=u(14:15);Hip_V=u(16:27);

LF_Q=Q(1:3);RF_Q=Q(4:6);RH_Q=Q(7:9);LH_Q=Q(10:12);

if tim<=1
    st_parameter=[Hip_Height;0;0;0;0;0;0;0;0;0];
    tim_sw_start=0;
    LF_LiftOff_P=[0;0;Hip_Height];RF_LiftOff_P=[0;0;Hip_Height];
    RH_LiftOff_P=[0;0;Hip_Height];LH_LiftOff_P=[0;0;Hip_Height];
    LF_init_Hip_Vxy=[0;0];
    RH_init_Hip_Vxy=[0;0];
    RF_init_Hip_Vxy=[0;0];
    LH_init_Hip_Vxy=[0;0];
    vx_des=0;
    vy_des=0;
else
%     vx_des=0;
    if tim<13
        vx_des=min(2.5,0.4*(tim-1));
    elseif tim>=13 && tim<=20
        vx_des=max(0,2.5-0.4*(tim-13));
    end
    %反向trot直接将vx_des取负
%     if tim<14
%         vx_des=max(-2.5,-0.3*(tim-1));
%     elseif tim>=14 && tim<=19
%         vx_des=min(0,-2.5+0.6*(tim-14));
%     end
    st_parameter=[Hip_Height;vx_des;vy_des;0;0;0;0;0;0;0];%10X1
end

if isequal(S,[3;0]) || isequal(S,[3.1;0]) || isequal(S,[3.2;0])|| isequal(S,[4;0])
   LF_LiftOff_P=Cal_LiftOff_footP(LF_Q);
%    RH_LiftOff_P=LF_LiftOff_P;
   RH_LiftOff_P=Cal_LiftOff_footP(RH_Q);
   tim_sw_start=tim;
   LF_init_Hip_Vxy=Hip_V(1:2);
   RH_init_Hip_Vxy=Hip_V(7:8);
elseif isequal(S,[1.1;2]) || isequal(S,[1.2;2]) || isequal(S,[1;2])
   RF_LiftOff_P=Cal_LiftOff_footP(RF_Q); 
%    LH_LiftOff_P=RF_LiftOff_P;
   LH_LiftOff_P=Cal_LiftOff_footP(LH_Q);
   tim_sw_start=tim;
   RF_init_Hip_Vxy=Hip_V(4:5);
   LH_init_Hip_Vxy=Hip_V(10:11);
end
Quad_LiftOff_P=[LF_LiftOff_P;RF_LiftOff_P;RH_LiftOff_P;LH_LiftOff_P];%12X1
Quad_init_Hip_V=[LF_init_Hip_Vxy;RF_init_Hip_Vxy;RH_init_Hip_Vxy;LH_init_Hip_Vxy];%8X1
pha_sw=(tim-tim_sw_start)/T_st;
pha_sw=min(1,pha_sw);%这一条很关键，防止摆动相结束后又摆回去。
sw_init_parameter=[Quad_LiftOff_P;Quad_init_Hip_V;pha_sw];
state_now=S(2);

y=[sw_init_parameter;st_parameter;state_now];
