function [delta_torque,delta_fy]=CalDelta_fy_torque(YAW,YAW_des,ROLL,ROLL_des)

global K_yaw C_yaw K_roll C_roll torso_width
yaw=YAW(1);yawd=YAW(2);yaw_des=YAW_des(1);yawd_des=YAW_des(2);
roll=ROLL(1);rolld=ROLL(2);roll_des=ROLL_des(1);rolld_des=ROLL_des(2);

M_yaw=K_yaw*(yaw_des-yaw) + C_yaw*(yawd_des-yawd);
delta_fy=M_yaw/torso_width;

M_roll=K_roll*(roll_des-roll) + C_roll*(rolld_des-rolld);
delta_torque=M_roll/2;

