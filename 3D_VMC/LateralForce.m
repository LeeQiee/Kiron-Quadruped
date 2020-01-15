function Torso_Force=LateralForce(u)
%要转换成空间力矢量，且表达在绝对坐标系下
xfb=u(1:13);tim=u(14);

if tim>=5 && tim<=5.25
    fp=[0;0;0];%-60;
else
    fp=[0;0;0];
end
qn = xfb(1:4);			% unit quaternion fixed-->f.b.
r = xfb(5:7);				% position of f.b. origin
Xa{6} = plux( rq(qn), r );		% xform fixed --> f.b. coords
X = inv(Xa{6});			% xform body i -> abs coords
pt = Xpt( X,[0;0;-0.1] );	% xform points to abs coords

Torso_Force=Fpt( fp, pt );
