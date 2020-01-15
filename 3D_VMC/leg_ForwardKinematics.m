function [P,Pd,J]=leg_ForwardKinematics(leg_Q,leg_Qd)

global L1 L2
q1=leg_Q(1);q2=leg_Q(2);q3=leg_Q(3);
% qd0=leg_Qd(1);qd1=leg_Qd(2);qd2=leg_Qd(3);

px=-L1*sin(q2)-L2*sin(q2+q3);
py=L1*sin(q1)*cos(q2)+L2*sin(q1)*cos(q2+q3);
pz=-L1*cos(q1)*cos(q2)-L2*cos(q1)*cos(q2+q3);
P=[px;py;pz];

J11=0;
J12=- (3*cos(q2 + q3))/5 - (3*cos(q2))/5;
J13=-(3*cos(q2 + q3))/5;
J21=-J12*cos(q1);
J22=- (3*sin(q1)*sin(q2))/5 - (3*sin(q2 + q3)*sin(q1))/5;
J23=-(3*sin(q2 + q3)*sin(q1))/5;
J31=-J12*sin(q1);
J32=(3*cos(q1)*sin(q2))/5 + (3*sin(q2 + q3)*cos(q1))/5;
J33=(3*sin(q2 + q3)*cos(q1))/5;
J=[J11 J12 J13;J21 J22 J23;J31 J32 J33];

Pd=J*leg_Qd;

