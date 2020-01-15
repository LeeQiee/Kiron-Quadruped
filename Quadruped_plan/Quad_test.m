function [LF_P_polar,LF_Pd_polar,LF_Jpolar]=Quad_test(LF_q1,LF_q2,LF_qd1,LF_qd2)

global L1 L2
LF_r=sqrt(L1^2+L2^2-2*L1*L2*cos(pi+LF_q2));
LF_theta=LF_q1-asin(L2*sin(pi+LF_q2)/LF_r);
LF_P_polar=[LF_r;LF_theta];


LF_J11=0;LF_J12=-(27*sin(LF_q2))/(250*((27*cos(LF_q2))/125 + 549/2500)^(1/2));
LF_J21=1;LF_J22=((3*cos(LF_q2))/(10*((27*cos(LF_q2))/125 + 549/2500)^(1/2)) + (81*sin(LF_q2)^2)/(2500*((27*cos(LF_q2))/125 + 549/2500)^(3/2)))/(1 - (9*sin(LF_q2)^2)/((108*cos(LF_q2))/5 + 549/25))^(1/2);
LF_Jpolar=[LF_J11,LF_J12;LF_J21,LF_J22];
LF_Pd_polar=LF_Jpolar*[LF_qd1;LF_qd2];

