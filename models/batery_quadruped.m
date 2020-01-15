function  model = batery_quadruped( npt )

% 电动四足模型建模

% For efficient use with Simulink, this function builds a new model only
% if it doesn't have a stored copy of the right model.

persistent last_model last_npt ;


if ~isempty(last_model) && last_npt == npt
  model = last_model;
  return
end

model.NB = 13;
model.parent = [0 1 2 3 1 5 6 1 8 9 1 11 12];

model.jtype = {'R','Rx','Ry','Ry','Rx','Ry','Ry','Rx','Ry','Ry','Rx','Ry','Ry'};% sacrificial joint replaced by floatbase
model.Xtree = {eye(6),xlt([0.85 0.35 -0.1]),xlt([0 0 0]),xlt([0 0 -0.6]),xlt([0.85 -0.35 -0.1]),xlt([0 0 0]),...
               xlt([0 0 -0.6]),xlt([-0.85 -0.35 -0.1]),xlt([0 0 0]),xlt([0 0 -0.6]),xlt([-0.85 0.35 -0.1]),...
               xlt([0 0 0]),xlt([0 0 -0.6])};

%重力
model.gravity =[0 0 -9.807];

%----- 惯量计算 ------%
%机身
torso_Ic=diag([0.53 2.93 3.38]);
torso_I=mcI(12,[0,0,0],torso_Ic);
%左前腿
LF_hip_Ic=diag([2.5e-5 1.667e-5 1.667e-5]); LF_hip_I=mcI(0.5,[0 0 0],LF_hip_Ic);
LF_thigh_Ic=diag([0.0001 0.015 0.015]);     LF_thigh_I=mcI(0.5,[0,0,-0.3],LF_thigh_Ic);
LF_shank_Ic=diag([0.0001 0.015 0.015]);     LF_shank_I=mcI(0.5,[0,0,-0.3],LF_shank_Ic);
% %机身
% torso_Ic=diag([17.5783  97.1783 112.1033]);
% torso_I=mcI(398,[0,0,0],torso_Ic);
% %左前腿
% LF_hip_Ic=diag([1.25e-6 3.33e-6 3.33e-6]); LF_hip_I=mcI(0.1,[0 0 0],LF_hip_Ic);
% LF_thigh_Ic=diag([0.0028 0.42 0.42]);       LF_thigh_I=mcI(14,[0,0,-0.3],LF_thigh_Ic);
% LF_shank_Ic=diag([0.0028 0.42 0.42]);    LF_shank_I=mcI(14,[0,0,-0.3],LF_shank_Ic);
% %右前腿
% RF_hip_Ic=diag([0.001369 0.001667 0.001667]); RF_hip_I=mcI(0.5,[0 0 0],RF_hip_Ic);
% RF_thigh_Ic=diag([0.027 0.027 0.001125]);     RF_thigh_I=mcI(2.5,[0,0,-0.18],RF_thigh_Ic);
% RF_shank_Ic=diag([0.0075 0.0075 0.00045]);    RF_shank_I=mcI(1,[0,0,-0.15],RF_shank_Ic);
% %左后腿
% LH_hip_Ic=diag([0.001369 0.001667 0.001667]); LH_hip_I=mcI(0.5,[0 0 0],LH_hip_Ic);
% LH_thigh_Ic=diag([0.027 0.027 0.001125]);     LH_thigh_I=mcI(2.5,[0,0,-0.18],LH_thigh_Ic);
% LH_shank_Ic=diag([0.0075 0.0075 0.00045]);    LH_shank_I=mcI(1,[0,0,-0.15],LH_shank_Ic);
% %右后腿
% RH_hip_Ic=diag([0.001369 0.001667 0.001667]); RH_hip_I=mcI(0.5,[0 0 0],RH_hip_Ic);
% RH_thigh_Ic=diag([0.027 0.027 0.001125]);     RH_thigh_I=mcI(2.5,[0,0,-0.18],RH_thigh_Ic);
% RH_shank_Ic=diag([0.0075 0.0075 0.00045]);    RH_shank_I=mcI(1,[0,0,-0.15],RH_shank_Ic);

model.I = {torso_I,LF_hip_I,LF_thigh_I,LF_shank_I,LF_hip_I,LF_thigh_I,LF_shank_I,LF_hip_I,...
           LF_thigh_I,LF_shank_I,LF_hip_I,LF_thigh_I,LF_shank_I};

% 三维绘图指令
%-----25度下坡地基-----%
% model.appearance.base = { 'tiles', [-1 32;-2 2;0 0],0.1};%正向trot
model.appearance.base = { 'tiles', [-32 1;-2 2;0 0],0.1,'colour', [1 0 0]};%反向trot,颜色修改不对
% model.appearance.base = { 'tiles', [-1 2;-2 2;0 0],0.1,... 
%                           'vertices', [7 2 -2.33;2 2 0;2 -2 0;7 -2 -2.33], ...
%                           'triangles', [1 2 3;3 4 1],...
%                           'tiles', [7 10;-2 2;-2.33 -2.33],0.1 };
% %-----25度上坡地基-----%
% model.appearance.base = { 'tiles', [-1 2;-2 2;0 0],0.1,... 
%                           'vertices', [7 2 2.33;2 2 0;2 -2 0;7 -2 2.33], ...
%                           'triangles', [1 2 3;3 4 1],...
%                           'tiles', [7 10;-2 2;2.33 2.33],0.1 };
% model.appearance.body{1} = { 'sphere', [0 0 0],0.05 };
model.appearance.body{1} = { 'box', [-0.85 -0.35 -0.1; 0.85 0.35 0.1],};
model.appearance.body{2} = { 'cyl', [-0.01 0 0; 0.01 0 0 ], 0.01 };
model.appearance.body{3} = { 'cyl', [0 0 0; 0 0 -0.6], 0.02 };
model.appearance.body{4} = { 'cyl', [0 0 0; 0 0 -0.6], 0.02 };

model.appearance.body{5} = { 'cyl', [-0.01 0 0; 0.01 0 0 ], 0.01 };
model.appearance.body{6} = { 'cyl', [0 0 0; 0 0 -0.6], 0.02 };
model.appearance.body{7} = { 'cyl', [0 0 0; 0 0 -0.6], 0.02 };

model.appearance.body{8} = { 'cyl', [-0.01 0 0; 0.01 0 0 ], 0.01 };
model.appearance.body{9} = { 'cyl', [0 0 0; 0 0 -0.6], 0.02 };
model.appearance.body{10} = { 'cyl', [0 0 0; 0 0 -0.6], 0.02 };

model.appearance.body{11} = { 'cyl', [-0.01 0 0; 0.01 0 0 ], 0.01 };
model.appearance.body{12} = { 'cyl', [0 0 0; 0 0 -0.6], 0.02 };
model.appearance.body{13} = { 'cyl', [0 0 0; 0 0 -0.6], 0.02 };

% 摄像设置
model.camera.body = 1;
model.camera.direction = [0 0.3 1.6];
model.camera.locus = [0 0.5];

% 地面接触点设置
model.gc.point = [ 0 0 0 0;0 0 0 0;-0.6 -0.6 -0.6 -0.6];
model.gc.body = [4 7 10 13 ];

% Final step: float the base

model = floatbase(model);		% replace joint 1 with a chain of 6
                                        % joints emulating a floating base
last_model = model;
last_npt = npt;