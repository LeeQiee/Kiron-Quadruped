function T=Forward_Kine_DH
% L1=0.6;
% L2=0.6;
syms q1 q2 q3 L1 L2 

% format short g
A01=transform(0,pi/2,0,pi/2);
A12=transform(0,pi/2,0,pi/2 +q1);
A23=transform(L1,0,0,pi+q2);
A34=transform(L2,0,0,q3);
T=A01*A12*A23*A34;
digits(4);%设置小数点位数
T=vpa(T);%转换成小数显示
T=simplify(T);%化简
end

function A=transform(a,alpha,d,theta)

A=[cos(theta)  -sin(theta)*cos(alpha)  sin(theta)*sin(alpha)  a*cos(theta);...
   sin(theta)  cos(theta)*cos(alpha)   -cos(theta)*sin(alpha) a*sin(theta);...
   0           sin(alpha)              cos(alpha)             d           ;...
   0           0                       0                      1];

end