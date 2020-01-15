function [LF_Height_des,RF_Height_des,RH_Height_des,LH_Height_des]=CalHipHeight(normal_height,Roll,Pitch)

global torso_width torso_length

roll_angle=Roll(1);pitch_angle=Pitch(1);

% LF_zdes=normal_height - torso_length*sin(pitch_angle)/2 +torso_width*sin(roll_angle)/2;
% RF_zdes=normal_height - torso_length*sin(pitch_angle)/2 -torso_width*sin(roll_angle)/2;
% RH_zdes=normal_height + torso_length*sin(pitch_angle)/2 -torso_width*sin(roll_angle)/2;
% LH_zdes=normal_height + torso_length*sin(pitch_angle)/2 +torso_width*sin(roll_angle)/2;
LF_zdes=normal_height ;
RF_zdes=normal_height ;
RH_zdes=normal_height ;
LH_zdes=normal_height ;
LF_Height_des=[0;0;LF_zdes];
RF_Height_des=[0;0;RF_zdes];
RH_Height_des=[0;0;RH_zdes];
LH_Height_des=[0;0;LH_zdes];

