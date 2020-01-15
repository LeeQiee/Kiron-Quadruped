function  [xdfb,tau] = OP_ID( model, xfb, q, qd, qdd, f_ext )

a_grav = get_gravity(model);

for i = 1:model.NB
  [ XJ, S{i} ] = jcalc( model.jtype{i}, q(i) );
  vJ = S{i}*qd(i);
  Xup{i} = XJ * model.Xtree{i};
%   if model.parent(i) ~=0       %计算惯性坐标系0到每个刚体坐标系的变换
%      X0up{i}=Xup{i}*X0up{model.parent(i)};
%   end
  if model.parent(i) == 0
    v{i} = vJ;
    c{i} = zeros(size(a_grav));		% spatial or planar zero vector
  else
    v{i} = Xup{i}*v{model.parent(i)} + vJ;
    c{i} = crm(v{i}) * vJ;
  end
  IA{i} = model.I{i};
  pA{i} = crf(v{i}) * model.I{i} * v{i};
end

for i=model.NB:-1:1
   D{i}=inv(S{i}'*IA{i}*S{i});
   K{i}=S{i}*D{i}*S{i}';
   LT{i}=eye(6)-IA{i}*K{i};
   if model.parent(i) ~=0
       IA{model.parent(i)}=IA{model.parent(i)}+Xup{i}'*LT{i}*IA{i}*Xup{i};
       pA{model.parent(i)}=pA{model.parent(i)}+Xup{i}'*LT{i}*(pA{i}+IA{i}*c{i});
       B{model.parent(i),i}=-Xup{i}'*IA{i}*S{i}*D{i};
       for j=1:length(model.root{i})
           B{model.parent(i),model.root{i}(j)}=Xup{i}'*LT{i}*B{i,model.root{i}(j)};
       end
   end   
end
JtT=ones(6,1);
for i=2:model.NB
    JtT=[JtT B{1,i}];
end
bt=pA{1}-IA{1}*(Xup{i}*a_grav + [zeros(3,1);skew(w{1})*v{1}]);


