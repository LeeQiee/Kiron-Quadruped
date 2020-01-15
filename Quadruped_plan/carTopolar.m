function [P_polar,Pd_polar]=carTopolar(P_cart,Pd_cart)
%----极坐标变换，给定笛卡尔坐标系下的位置速度，计算极坐标下的位置、速度变量----%

px=P_cart(1);
pz=P_cart(2);
% RF_px=RF_P_cart(1);
% RF_py=RF_P_cart(2);

r_polar=sqrt(px^2+pz^2);
theta_polar=atan(px/pz);
P_polar=[r_polar;theta_polar];

% RF_r_polar=sqrt(RF_px^2+RF_py^2);
% RF_theta_polar=atan(RF_px/RF_py);
% RF_P_polar=[RF_r_polar;RF_theta_polar];

J=[px/r_polar,pz/r_polar;pz/(r_polar^2),-px/(r_polar^2)];
Pd_polar=J*Pd_cart;

% RF_J=[RF_px/RF_r_polar,RF_py/RF_r_polar;RF_py/(RF_r_polar^2),-RF_px/(RF_r_polar^2)];
% RF_Pd_polar=RF_J*RF_Pd_cart;