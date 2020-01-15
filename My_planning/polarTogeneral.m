function y=polarTogeneral(u)

global L_leg
r=u(1);
theta=u(2);

q1=theta-acos(0.5*r/L_leg);
q2=2*acos(0.5*r/L_leg);
y=[q1;q2];

