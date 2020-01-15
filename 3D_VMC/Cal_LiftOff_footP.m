function LiftOff_P=Cal_LiftOff_footP(leg_Q)

global L1 L2
q1=leg_Q(1);q2=leg_Q(2);q3=leg_Q(3);

px=-L1*sin(q2)-L2*sin(q2+q3);
py=L1*sin(q1)*cos(q2)+L2*sin(q1)*cos(q2+q3);
pz=-L1*cos(q1)*cos(q2)-L2*cos(q1)*cos(q2+q3);
LiftOff_P=[px;py;pz];
