function Quad_traj_test

pha_sw=0:0.01:0.5;
[P_car_des,Pd_car_des]=Quad_Swing_traj(pha_sw);
figure(1)
plot (P_car_des(1,:),P_car_des(2,:));
% figure(2)
% plot (Pd_car_des(1,:),Pd_car_des(2,:));
hold on

pha_sw=0.5:0.01:1;
[P_car_des,Pd_car_des]=Quad_Swing_traj(pha_sw);
figure(1)
plot (P_car_des(1,:),P_car_des(2,:));
% figure(2)
% plot (Pd_car_des(1,:),Pd_car_des(2,:));
hold on
for i=0:100
    pha_st=i/100;
    [P_car_des,Pd_car_des]=Quad_Stance_traj(pha_st);
    px(i+1)=P_car_des(1);
    pz(i+1)=P_car_des(2);
end
figure(1)
plot (px,pz);
% xlim([-0.3,0.3]);
% ylim([-0.5,0]);
