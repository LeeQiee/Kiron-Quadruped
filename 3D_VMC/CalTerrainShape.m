function z=CalTerrainShape(x)

b=0.25;

if x<=3
    z=0;
elseif x>2 && x<4.5
    z=0.04;
%     delta_x=mod(x-3,b);
%     z=period(delta_x);
elseif x>=5.5
    z=0;
end
end
     
function z=period(delta_x) 

h=0.03;

if delta_x>=0 && delta_x<=h
    z=delta_x;
elseif delta_x>h && delta_x<=2*h
    z=h;
elseif delta_x>2*h && delta_x<=3*h
    z=3*h-delta_x;
elseif delta_x>3*h
    z=0;
end
end

% function Terraintest
% for i=1:1:1000
%     x(i)=i/100;
%     y(i)=CalTerrainShape(x(i));
% end
% % x=0:0.01:10;
% % plot(x,z);
% plot(x,y);
% axis([3 6 0 1]);%设置坐标轴范围要在plot函数之后 
% end

