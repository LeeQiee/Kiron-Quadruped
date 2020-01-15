function  y = gcFD( model, FDfun, u )

% gcFD  Simulink wrapper for forward dynamics functions
% y=gcFD(model,FDfun,u)  is a Simulink wrapper for the forward-dynamics
% functions FDab, FDcrb, FDfb and FDgq.  It serves two purposes: (1) to
% accept ground contact forces in the format produced by the spatial_v2
% Simulink ground model and convert them into the format required by the
% dynamics functions, and (2) to concatenate into a single vector the two
% return values of FDfb.  The first argument is the model data structure;
% the second is the function handle of the dynamics function to be used;
% and the third is the concatenation of the vectors q, qd, tau and
% (optionally) the ground contact force data (f_gc) from which f_ext can
% be calculated.  (For FDfb, u is the concatenation of x, q, qd, tau and
% optional f_gc.)  The ground contact force data, if supplied, is simply
% the concatenation of the spatial (or planar) forces arising from each
% point defined in model.gc.point, all expressed in base coordinates.

N = model.NB;

if isequal( FDfun, @FDfb )
  x = u(1:13);
  q = u(14:N+7);
  qd = u(N+8:2*N+1);
  tau = u(2*N+2:3*N-5);
  f_gc_torso = u(3*N-4:end);%抗侧向冲击验证
  f_gc = f_gc_torso(1:24);%抗侧向冲击验证
  f_torso=f_gc_torso(25:30);%抗侧向冲击验证
else
  q = u(1:N);
  qd = u(N+1:2*N);
  tau = u(2*N+1:3*N);
  f_gc = u(3*N+1:end);
end

if ~isempty(f_gc)
  vecsize = length(model.Xtree{1});	% 3 for planar, 6 for spatial
  b = model.gc.body;			% b(i) == body number of point i
  f_gc = reshape( f_gc, vecsize, length(b) );
  f_ext = cell(1,N);
  % for each body i that is mentioned at least once, calculate the sum of
  % all forces acting on that body
  for i = unique(b)
    f_ext{i} = sum( f_gc(:,b==i), 2 );
  end
  f_ext{6}=f_torso;%抗侧向冲击验证
else
  f_ext = {};
  f_ext{6}=f_torso;%抗侧向冲击验证
end

if isequal( FDfun, @FDfb )
  [xd, qdd] = FDfb( model, x, q, qd, tau, f_ext );
  y = [xd; qdd];
else
  y = FDfun( model, q, qd, tau, f_ext );
end
