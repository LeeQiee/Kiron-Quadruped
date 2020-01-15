function  model = singleg( npt )

% 单腿模型建模

% For efficient use with Simulink, this function builds a new model only
% if it doesn't have a stored copy of the right model.

persistent last_model last_npt;

if length(last_model) ~= 0 && last_npt == npt
  model = last_model;
  return
end

model.NB = 3;
model.parent = [0 1 2];

model.jtype = {'R','R','R'};		% sacrificial joint replaced by floatbase
model.Xtree = {eye(6),plux(ry(pi/2),[0 0 0]),xlt([0.5 0 0])};%将plux中的[0,0,-0.15]修改为[0,0,0]

%重力
model.gravity = [0 0 -9.80665];

% 惯量计算
torso_Ic=diag([0.73 0.34 0.89]);
thigh_Ic=diag([0.0002 0.02083 0.02083]);
shank_Ic=diag([0.0002 0.02083 0.02083]);

torso_I=mcI(12,[0,0,0],torso_Ic);
thigh_I=mcI(1,[0.25,0,0],thigh_Ic);
shank_I=mcI(1,[0.25,0,0],shank_Ic);

model.I = {torso_I,thigh_I,shank_I};

% 三维绘图指令
model.appearance.base = { 'tiles', [ -0.8 0.8;-0.5 3; 0 0], 0.1};
% model.appearance.body{1} = { 'sphere', [0 0 0],0.05 };
model.appearance.body{1} = { 'box', [-0.25 -0.4 -0.15; 0.25 0.4 0.15] };
model.appearance.body{2} = { 'cyl', [0 0 0; 0.5 0 0], 0.02 };
model.appearance.body{3} = { 'cyl', [0 0 0; 0.5 0 0], 0.02 };

% 摄像设置
% model.camera.body = 1;
% model.camera.direction = [-0.5 0.3 1.6];
% model.camera.locus = [0 0.5];

% 地面接触点设置
model.gc.point = [ 0.5;0;0 ];
model.gc.body = 3;

% Final step: float the base

model = floatbase(model);		% replace joint 1 with a chain of 6
                                        % joints emulating a floating base
last_model = model;
last_npt = npt;