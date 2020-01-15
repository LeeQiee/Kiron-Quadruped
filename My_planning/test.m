function test
i=1;
for tim=0.4:0.001:0.6
    [P1_des,Pd1_des]=Stance_tra(0.4,tim);
    [P_polar,Pd_polar]=polar_transform(P1_des,Pd1_des);
    px(i)=P1_des(1,1);
    py(i)=P1_des(2,1);
    pdx(i)=Pd1_des(1,1);
    pdy(i)=Pd1_des(2,1);
    r(i)=P_polar(1,1);
    theta(i)=P_polar(2,1);
    rd(i)=Pd_polar(1,1);
    thetad(i)=Pd_polar(2,1);
    i=i+1;
end 
% plot(px,py);
% plot(theta,r);
plot(rd);
hold on
plot(thetad);

    