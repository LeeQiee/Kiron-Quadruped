function  model = quadruped( npt )

% 四足模型建模

% For efficient use with Simulink, this function builds a new model only
% if it doesn't have a stored copy of the right model.

persistent last_model last_npt;

if ~isempty(last_model) && last_npt == npt
  model = last_model;
  return
end

model.NB = 13;
model.parent = [0 1 2 3 1 5 6 1 8 9 1 11 12];

model.jtype = {'R','Rx','Ry','Ry','Rx','Ry','Ry','Rx','Ry','Ry','Rx','Ry','Ry'};% sacrificial joint replaced by floatbase
model.Xtree = {eye(6),xlt([0.5 0.3 -0.1]),xlt([0 0 -0.074]),xlt([0 0 -0.36]),xlt([0.5 -0.3 -0.1]),xlt([0 0 -0.074]),...
               xlt([0 0 -0.36]),xlt([-0.5 -0.3 -0.1]),xlt([0 0 -0.074]),xlt([0 0 -0.36]),xlt([-0.5 0.3 -0.1]),...
               xlt([0 0 -0.074]),xlt([0 0 -0.36])};

%重力
model.gravity = [0 0 -9.80665];

%----- 惯量计算 ------%
%机身
torso_Ic=diag([2 5.2 6.8]);
torso_I=mcI(60,[0,0,0],torso_Ic);
%左前腿
LF_hip_Ic=diag([0.001369 0.000417 0.000417]); LF_hip_I=mcI(0.5,[0 0 0],LF_hip_Ic);
LF_thigh_Ic=diag([0.027 0.027 0.0005]);       LF_thigh_I=mcI(2.5,[0,0,-0.18],LF_thigh_Ic);
LF_shank_Ic=diag([0.0075 0.0075 0.0002]);    LF_shank_I=mcI(1,[0,0,-0.15],LF_shank_Ic);
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
model.appearance.base = { 'box', [-1 -1 0;4 1 -0.001],... 
                          'vertices', [4 1 0.352;2 1 0;2 -1 0;4 -1 0.352], ...
                          'triangles', [1 2 3;3 4 1],...
                          'box', [4 -1 0.352;10 1 0.351] };
% model.appearance.body{1} = { 'sphere', [0 0 0],0.05 };
model.appearance.body{1} = { 'box', [-0.5 -0.3 -0.1; 0.5 0.3 0.1] };
model.appearance.body{2} = { 'cyl', [-0.05 0 0; 0.05 0 0 ], 0.074 };
model.appearance.body{3} = { 'cyl', [0 0 0; 0 0 -0.36], 0.02 };
model.appearance.body{4} = { 'cyl', [0 0 0; 0 0 -0.3], 0.02 };

model.appearance.body{5} = { 'cyl', [-0.05 0 0; 0.05 0 0 ], 0.074 };
model.appearance.body{6} = { 'cyl', [0 0 0; 0 0 -0.36], 0.02 };
model.appearance.body{7} = { 'cyl', [0 0 0; 0 0 -0.3], 0.02 };

model.appearance.body{8} = { 'cyl', [-0.05 0 0; 0.05 0 0 ], 0.074 };
model.appearance.body{9} = { 'cyl', [0 0 0; 0 0 -0.36], 0.02 };
model.appearance.body{10} = { 'cyl',[0 0 0; 0 0 -0.3], 0.02 };

model.appearance.body{11} = { 'cyl', [-0.05 0 0; 0.05 0 0 ], 0.074 };
model.appearance.body{12} = { 'cyl', [0 0 0; 0 0 -0.36], 0.02 };
model.appearance.body{13} = { 'cyl', [0 0 0; 0 0 -0.3], 0.02 };

% 摄像设置
model.camera.body = 1;
model.camera.direction = [0 0.3 1.6];
model.camera.locus = [0 0.5];

% 地面接触点设置
model.gc.point = [ 0 0 0 0;0 0 0 0;-0.3 -0.3 -0.3 -0.3 ];
model.gc.body = [4 7 10 13];

% Final step: float the base

model = floatbase(model);		% replace joint 1 with a chain of 6
                                        % joints emulating a floating base
last_model = model;
last_npt = npt;