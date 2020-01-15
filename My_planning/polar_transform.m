function [P_polar,Pd_polar]=polar_transform(P_cart,Pd_cart)
%----极坐标变换，给定笛卡尔坐标系下的位置速度，计算极坐标下的位置、速度变量----%

px=P_cart(1,1);
py=P_cart(2,1);
r_polar=sqrt(px*px+py*py);
theta_polar=atan(px/py);
P_polar=[r_polar;theta_polar];

J=[px/r_polar,py/r_polar;py/(r_polar*r_polar),-px/(r_polar*r_polar)];
Pd_polar=J*Pd_cart;