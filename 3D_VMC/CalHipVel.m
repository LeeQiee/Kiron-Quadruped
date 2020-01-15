function Hip_V=CalHipVel(model, xfb, q, qd)

  qn = xfb(1:4);			% unit quaternion fixed-->f.b.
  r = xfb(5:7);				% position of f.b. origin
  Xa{6} = plux( rq(qn), r );		% xform fixed --> f.b. coords

  vfb = xfb(8:end);
  vb{6} = Xa{6} * vfb;			% f.b. vel in f.b. coords

  for i = 7:model.NB
    [ XJ, S ] = jcalc( model.jtype{i}, q(i-6) );
    Xup = XJ * model.Xtree{i};
    vJ = S*qd(i-6);
    Xa{i} = Xup * Xa{model.parent(i)};
    vb{i} = Xup * vb{model.parent(i)} + vJ;
  end
  %LF腿髋关节，对应刚体8,还要加上浮动基座的六个虚拟刚体
  X = inv(Xa{8});			% xform body i -> abs coords
  v = X * vb{8};			% body i vel in abs coords
  pt = Xpt( X, [0; 0; 0] );	% xform points to abs coords
  LF_HipV= Vpt( v, pt );			% linear velocities of points
  
  %RF腿髋关节，对应刚体11
  X = inv(Xa{11});			
  v = X * vb{11};			
  pt = Xpt( X, [0; 0; 0] );	
  RF_HipV= Vpt( v, pt );	
  
  %RH腿髋关节，对应刚体14
  X = inv(Xa{14});			
  v = X * vb{14};			
  pt = Xpt( X, [0; 0; 0] );	
  RH_HipV= Vpt( v, pt );
  
  %LH腿髋关节，对应刚体17
  X = inv(Xa{17});			
  v = X * vb{17};			
  pt = Xpt( X, [0; 0; 0] );	
  LH_HipV= Vpt( v, pt );
  
  Hip_V=[LF_HipV;RF_HipV;RH_HipV;LH_HipV];
  