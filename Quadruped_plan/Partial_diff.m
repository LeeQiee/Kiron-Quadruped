% L0=0.074;
L1=0.6;
L2=0.6;
% syms q1 q2
% r=sqrt(L1^2+L2^2-2*L1*L2*cos(pi+q2));
% theta=q1-asin(L2*sin(pi+q2)/r);
% r=sqrt(L1^2+L2^2-2*L1*L2*cos(pi-q2));
% theta=q1+asin(L2*sin(pi-q2)/r);
% J11=diff(r,q1,1);
% J12=diff(r,q2,1);
% J21=diff(theta,q1,1);
% J22=diff(theta,q2,1);
syms q1 q2 q3
px=-L1*sin(q2)-L2*sin(q2+q3);
py=L1*sin(q1)*cos(q2)+L2*sin(q1)*cos(q2+q3);
pz=-L1*cos(q1)*cos(q2)-L2*cos(q1)*cos(q2+q3);
J11=diff(px,q1,1);J12=diff(px,q2,1);J13=diff(px,q3,1);
J21=diff(py,q1,1);J22=diff(py,q2,1);J23=diff(py,q3,1);
J31=diff(pz,q1,1);J32=diff(pz,q2,1);J33=diff(pz,q3,1);

% syms roll
% q0=-roll-acos((1-0.35*tan(roll))/1);
% q1=-roll+acos((1+0.35*tan(roll))/1);
% J11=diff(q0,roll,1);
% J12=diff(q1,roll,1);

% syms r theta
% q2=acos((L1^2+L2^2-r^2)/(2*L1*L2))-pi;
% q1=asin((L2*sin(pi+q2))/r)+theta;
% J11=diff(q1,r,1);
% J12=diff(q1,theta,1);
% J21=diff(q2,r,1);
% J22=diff(q2,theta,1);


% syms r theta
% L_AB=sqrt(L0^2 +r^2 -2*L0*r*cos(theta));
% q2=acos((L1^2+L2^2-L_AB^2)/(2*L1*L2))-pi;
% q1=asin(r*sin(theta)/L_AB)+acos((L1^2+L_AB^2-L2^2)/(2*L1*L_AB));
% J11=diff(q1,r,1);
% J12=diff(q1,theta,1);
% J21=diff(q2,r,1);
% J22=diff(q2,theta,1);






