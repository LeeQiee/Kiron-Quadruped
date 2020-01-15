function cycloid_curve_swing_test

T_st=0.5; 
SwingHeight=0.05;
pha_sw=0:0.01:1;
xlo=-0.1;ylo=0;zlo=-1;
xf=0.1;yf=0.1;
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
figure (1)
plot(x_des,z_des);
% figure (2)
% plot3(x_des,y_des,z_des);
% figure (3)
% plot(xd_des,yd_des);
% figure (4)
% scatter3(xd_des,yd_des,zd_des,'k*');%»­ÈýÎ¬É¢µãÍ¼
