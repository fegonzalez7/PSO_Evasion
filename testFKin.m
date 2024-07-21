% Robot
escala = 1e-3;
l = escala*[70 352 360 380 65];
qlims = deg2rad([-180 180; -90 110; -230 49; -200 200; -115 115; -400 400]);
L(1) = Link('revolute','alpha', 0,    'a',0,   'd',l(2),'offset', 0,   'qlim',qlims(1,:),'modified');
L(2) = Link('revolute','alpha',-pi/2, 'a',l(1),'d',0,   'offset',-pi/2,'qlim',qlims(2,:),'modified');
L(3) = Link('revolute','alpha', 0,    'a',l(3),'d',0,   'offset', 0,   'qlim',qlims(3,:),'modified');
L(4) = Link('revolute','alpha',-pi/2, 'a',0,   'd',l(4),'offset', 0,   'qlim',qlims(4,:),'modified');
L(5) = Link('revolute','alpha', pi/2, 'a',0,   'd',0,   'offset', 0,   'qlim',qlims(5,:),'modified');
L(6) = Link('revolute','alpha',-pi/2, 'a',0,   'd',0,   'offset', pi,   'qlim',qlims(6,:),'modified');
R = SerialLink(L,'name','ABB IRB140');
R.tool= transl(0,0,l(5));
%%
syms q1 q2 q3 q4 q5 q6 as real
%%
A01 = simplify(L(1).A(q1));
%%
A12 = simplify(L(2).A(q2));
A12 = MTH_simplex(A12);
%%
A23 = simplify(L(3).A(q3));
%%
A34 = simplify(L(4).A(q4));
A34 = MTH_simplex(A34);
%%
A45 = simplify(L(5).A(q5));
A45 = MTH_simplex(A45);
%%
A56 = simplify(L(6).A(q6));
A56 = MTH_simplex(A56);
%%
seq = [1 2 3];
q = [0 0 0];
T = eye(4);
l = 0.001*[70 352 360 380 65];
for i=1:numel(seq)
   switch seq(i)
      case 1
         Taux = [cos(q(i)) -sin(q(i)) 0 0;
                 sin(q(i))  cos(q(i)) 0 0;
                 0          0         1 l(2);
                 0          0         0 1];
      case 2
         Taux = [sin(q(i))  cos(q(i)) 0 l(1);
                 0          0         1 0;
                 cos(q(i)) -sin(q(i)) 0 0;
                 0          0         0 1];
      case 3
         Taux = [cos(q(i)) -sin(q(i)) 0 l(3);
                 sin(q(i))  cos(q(i)) 0 0;
                 0          0         1 0;
                 0          0         0 1];
      case 4
         Taux = [cos(q(i)) -sin(q(i)) 0 0;
                 0          0         1 l(4);
                -sin(q(i)) -cos(q(i)) 0 0;
                 0          0         0 1];
      case 5
         Taux = [cos(q(i)) -sin(q(i)) 0 0;
                 0          0        -1 0;
                 sin(q(i))  cos(q(i)) 0 0;
                 0          0         0 1];
      case 6
         Taux = [-cos(q(i))  sin(q(i)) 0 0;
                  0          0         1 0;
                  sin(q(i))  cos(q(i)) 0 0;
                  0          0         0 1];
      otherwise
         Taux = eye(4);
   end
   T = T*Taux;
end



