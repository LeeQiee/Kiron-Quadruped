%-----Simulink仿真所需的全局控制参数-----%
global K_yaw C_yaw K_roll C_roll torso_width Hip_Height SwingHeight L1 L2 ...
       K_stance C_stance K_swing C_swing T_st K_vx K_vy vx_des vy_des torso_length
   
T_st=0.35;
Hip_Height=-1;
vx_des=0;
vy_des=0;

K_stance=diag([0 0 5000]); 
K_swing=diag([1200 1200 6000]); 
C_stance=diag([150 50 100]);  
C_swing=diag([50 50 120]); 
K_yaw=300;
K_roll=300;
C_yaw=50;
C_roll=50;
K_vx=0.02;
K_vy=0.02;

torso_width=0.7;
torso_length=1.7;
SwingHeight=0.09;%抬腿高度

L1=0.6;
L2=0.6;
clear GaitGenerator;