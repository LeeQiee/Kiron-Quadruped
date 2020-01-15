function test1

key_point_y=[0.8775;0.78;-0.5655;0;0;0];
key_point_x=[-0.2;0;-2;2;0;0];
tf=0.5;

 X = [
    0           0           0         0       0   1
    tf^5        tf^4        tf^3      tf^2    tf  1
    0           0           0         0       1   0
    5*tf^4      4*tf^3      3*tf^2    2*tf    1   0
    0           0           0         2       0   0
    20*tf^3     12*tf^2     6*tf      2       0   0
 ];
 y_coeffs = (X \ key_point_y)';%左除相当于inv(X)*key_point_y
 x_coeffs = (X \ key_point_x)';
 x_coeffs_d = x_coeffs(1:5) .* (5:-1:1);
y_coeffs_d = y_coeffs(1:5) .* (5:-1:1);
 m=1;
 for t=0:0.001:1
     tim(m)=t;
     if t<=0.5
         px(m)=polyval(x_coeffs,t); 
         py(m)=polyval(y_coeffs,t);
         pdx(m)=polyval(x_coeffs_d,t); 
         pdy(m)=polyval(y_coeffs_d,t);
     else
         px(m)=-polyval(x_coeffs,1-t); 
         py(m)=polyval(y_coeffs,1-t);
         pdx(m)=polyval(x_coeffs_d,1-t); 
         pdy(m)=-polyval(y_coeffs_d,1-t);
     end
     m=m+1;
 end
 plot(pdx,pdy);
 hold on;
 
% key_point_x=[0;0.2;2;-2;0;0];
% key_point_y=[0.78;0.8775;0;0.5655;0;0];
% t0=0.5;
% tf=1;
% 
%  X = [
%     t0^5        t0^4        t0^3      t0^2    t0  1
%     tf^5        tf^4        tf^3      tf^2    tf  1
%     5*t0^4      4*t0^3      3*t0^2    2*t0    1   0
%     5*tf^4      4*tf^3      3*tf^2    2*tf    1   0
%     20*t0^3     12*t0^2     6*t0      2       0   0
%     20*tf^3     12*tf^2     6*tf      2       0   0
%  ];
%  y_coeffs = (X \ key_point_y)';%左除相当于inv(X)*key_point_y
%  x_coeffs = (X \ key_point_x)';
%   x_coeffs_d = x_coeffs(1:5) .* (5:-1:1);
% y_coeffs_d = y_coeffs(1:5) .* (5:-1:1);
% 
%  m=1;
%  for t=0.5:0.001:1
%      tim(m)=t;
%      px(m)=polyval(x_coeffs,t); 
%      py(m)=polyval(y_coeffs,t);
%      pdx(m)=polyval(x_coeffs_d,t); 
%      pdy(m)=polyval(y_coeffs_d,t);
%      m=m+1;
%  end
%  plot(pdx,pdy);
 
 