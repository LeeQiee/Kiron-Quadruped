function [Leg_Torque,Hip_Force]=CalLegTorque(P_des,Pd_des,leg_Q,leg_Qd,delta_fx,Hip_V)

global K_stance C_stance K_swing C_swing

[P,Pd,J]=leg_ForwardKinematics(leg_Q,leg_Qd);
if nargin==6 %站立相
    Hip_P=[0;0;-P(3)];
    F=K_stance*(P_des-Hip_P)+C_stance*(Pd_des-Hip_V)+[delta_fx;0;0];
    F(2)=min(20,F(2));
    F(2)=max(-20,F(2));
%     F(1)=min(20,F(1));
%     F(1)=max(-20,F(1));
%     F=[-varF(1);varF(2);varF(3)];
elseif nargin==4 %摆动相
    F=K_swing*(P_des-P)+C_swing*(Pd_des-Pd);
    F(2)=min(20,F(2));
    F(2)=max(-20,F(2));
%     F=varF;
    
end
Hip_Force=F;
Leg_Torque=J'*F;
