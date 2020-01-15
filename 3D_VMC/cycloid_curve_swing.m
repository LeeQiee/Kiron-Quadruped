function [swingP_des,swingPd_des]=cycloid_curve_swing(pha_sw,initPfoot,initHip_V,initHip_V_des)

global T_st K_vx K_vy SwingHeight
initHip_vx=initHip_V(1);initHip_vy=initHip_V(2);
initHip_vx_des=initHip_V_des(1);initHip_vy_des=initHip_V_des(2);
xlo=initPfoot(1);ylo=initPfoot(2);zlo=initPfoot(3);

xf=0.5*initHip_vx*T_st - K_vx*(initHip_vx_des-initHip_vx);
xf=min(0.3,xf);
yf=0.5*initHip_vy*T_st - K_vy*(initHip_vy_des-initHip_vy);
yf=min(0.25,yf);
pha=2*pi*pha_sw;
var1=(pha-sin(pha))/(2*pi);
var2=(1-cos(pha))/2;
pha_d=2*pi/T_st;

x_des=(xf-xlo)*var1+xlo;
y_des=(yf-ylo)*var1+ylo;
z_des=SwingHeight*var2+zlo;

xd_des=pha_d*(xf-xlo)*var2/pi;
yd_des=pha_d*(yf-ylo)*var2/pi;
zd_des=pha_d*SwingHeight*sin(pha)/2;

swingP_des=[x_des;y_des;z_des];
swingPd_des=[xd_des;yd_des;zd_des];
